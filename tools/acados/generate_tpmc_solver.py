"""Generate the C implementation of the acados TMPC solver."""

import argparse
import os
from pathlib import Path
import shutil
import sys

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
ACADOS_TEMPLATE_DIRECTORY = (
    REPOSITORY_ROOT / "third_party" / "acados" / "interfaces" / "acados_template"
)
if str(ACADOS_TEMPLATE_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(ACADOS_TEMPLATE_DIRECTORY))

from acados_template import AcadosOcpSolver

from tpmc_acados_model import create_tpmc_ocp


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=Path("build/acados_tpmc"),
        help="Directory receiving generated acados C code.",
    )
    return parser.parse_args()


def configure_acados_environment(repository_root: Path) -> None:
    acados_root = repository_root / "third_party" / "acados"
    os.environ.setdefault("ACADOS_SOURCE_DIR", str(acados_root))
    os.environ.setdefault("ACADOS_INSTALL_DIR", str(acados_root))

    renderer_path = os.environ.get("TERA_PATH")
    if renderer_path is None:
        installed_renderer = acados_root / "bin" / "t_renderer"
        local_renderer = (
            acados_root / "interfaces" / "acados_template" / "tera_renderer" /
            "target" / "release" / "t_renderer"
        )
        renderer_path = str(
            installed_renderer if installed_renderer.is_file() else local_renderer
        )
        os.environ["TERA_PATH"] = renderer_path

    if not Path(renderer_path).is_file():
        raise RuntimeError(
            "acados code generation requires a pre-installed t_renderer. "
            f"Set TERA_PATH to a local executable; not found: {renderer_path}"
        )

    library_directory = str(acados_root / "lib")
    library_path = os.environ.get("LD_LIBRARY_PATH", "")
    library_entries = library_path.split(os.pathsep) if library_path else []
    if library_directory in library_entries:
        return

    # The generated solver is loaded by ctypes after code generation.  The
    # dynamic loader reads LD_LIBRARY_PATH at process startup, so restart once
    # with acados' local libraries included instead of requiring shell setup.
    if os.environ.get("TMPC_ACADOS_LIBRARY_PATH_READY") != "1":
        environment = os.environ.copy()
        environment["LD_LIBRARY_PATH"] = os.pathsep.join(
            (library_directory, *library_entries)
        )
        environment["TMPC_ACADOS_LIBRARY_PATH_READY"] = "1"
        os.execvpe(sys.executable, [sys.executable, *sys.argv], environment)


def write_solver_bridge(
    output_directory: Path,
    nlp_solver_type: str,
    rti_log_residuals: bool,
    external_qp_residuals: bool,
) -> None:
    solver_headers = sorted(output_directory.glob("acados_solver_ocp_tpmc_*.h"))
    if len(solver_headers) != 1:
        raise RuntimeError(
            "Expected exactly one generated TMPC solver header, found "
            f"{len(solver_headers)} in {output_directory}"
        )

    header = solver_headers[0]
    generated_prefix = header.stem.removeprefix("acados_solver_")
    capsule_type = f"{generated_prefix}_solver_capsule"
    bridge = f"""#pragma once

#include \"{header.name}\"

namespace mpc_controller::tpmc::generated {{

inline constexpr const char kNlpSolverType[] = "{nlp_solver_type}";
inline constexpr bool kUsesSqpRti = {str(nlp_solver_type == "SQP_RTI").lower()};
inline constexpr bool kRtiLogsResiduals = {str(rti_log_residuals).lower()};
inline constexpr bool kHasExternalQpResiduals = {str(external_qp_residuals).lower()};

using SolverCapsule = {capsule_type};

inline SolverCapsule *createCapsule() noexcept {{
  return {generated_prefix}_acados_create_capsule();
}}

inline int freeCapsule(SolverCapsule *capsule) noexcept {{
  return {generated_prefix}_acados_free_capsule(capsule);
}}

inline int create(SolverCapsule *capsule) noexcept {{
  return {generated_prefix}_acados_create(capsule);
}}

inline int resetSolverState(SolverCapsule *capsule) noexcept {{
  return {generated_prefix}_acados_reset(capsule, 1, 0, 0, 0);
}}

inline int updateParameters(SolverCapsule *capsule, int stage, double *values,
                            int parameter_count) noexcept {{
  return {generated_prefix}_acados_update_params(
      capsule, stage, values, parameter_count);
}}

inline int solve(SolverCapsule *capsule) noexcept {{
  return {generated_prefix}_acados_solve(capsule);
}}

inline int freeSolver(SolverCapsule *capsule) noexcept {{
  return {generated_prefix}_acados_free(capsule);
}}

inline ocp_nlp_config *config(SolverCapsule *capsule) noexcept {{
  return {generated_prefix}_acados_get_nlp_config(capsule);
}}

inline ocp_nlp_dims *dimensions(SolverCapsule *capsule) noexcept {{
  return {generated_prefix}_acados_get_nlp_dims(capsule);
}}

inline ocp_nlp_in *input(SolverCapsule *capsule) noexcept {{
  return {generated_prefix}_acados_get_nlp_in(capsule);
}}

inline ocp_nlp_out *output(SolverCapsule *capsule) noexcept {{
  return {generated_prefix}_acados_get_nlp_out(capsule);
}}

inline ocp_nlp_solver *solver(SolverCapsule *capsule) noexcept {{
  return {generated_prefix}_acados_get_nlp_solver(capsule);
}}

}}  // namespace mpc_controller::tpmc::generated
"""
    (output_directory / "tpmc_generated_solver_bridge.hpp").write_text(bridge)


def reset_generated_output(
    repository_root: Path, output_directory: Path
) -> Path:
    """Reset only the repository's dedicated generated-solver directory."""

    resolved_output = output_directory.resolve()
    generated_root = (repository_root / "build").resolve()
    if resolved_output.parent != generated_root:
        raise RuntimeError(
            "The generated solver directory must be directly under "
            f"{generated_root}: {resolved_output}"
        )
    if resolved_output.exists():
        shutil.rmtree(resolved_output)
    resolved_output.mkdir(parents=True)
    return resolved_output


def generate_solver(output_directory: Path) -> None:
    configure_acados_environment(REPOSITORY_ROOT)

    output_directory = reset_generated_output(REPOSITORY_ROOT, output_directory)
    ocp = create_tpmc_ocp(output_directory)
    AcadosOcpSolver(ocp, build=False, generate=True)
    write_solver_bridge(
        output_directory,
        ocp.solver_options.nlp_solver_type,
        bool(ocp.solver_options.rti_log_residuals),
        bool(ocp.solver_options.nlp_solver_ext_qp_res),
    )


def main() -> None:
    arguments = parse_arguments()
    generate_solver(arguments.output_directory)
    print(f"Generated acados TMPC solver in {arguments.output_directory.resolve()}")


if __name__ == "__main__":
    main()
