#include "tinympc/types.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#ifndef TINYMPC_API_HEADER_PATH
#error "TINYMPC_API_HEADER_PATH must point to the pinned TinyMPC API header"
#endif

namespace {

bool contains(const std::string &source, const std::string &needle)
{
  return source.find(needle) != std::string::npos;
}

}  // namespace

int main()
{
  std::ifstream api_header(TINYMPC_API_HEADER_PATH);
  if (!api_header) {
    std::cerr << "cannot read pinned TinyMPC API header\n";
    return 1;
  }

  std::ostringstream contents;
  contents << api_header.rdbuf();
  const std::string api = contents.str();

  // These checks are deliberately source-backed. If the pinned API gains
  // either capability, this test fails and forces a fresh parity review
  // before anyone enables a production backend.
  const bool has_stage_dynamics_api =
    contains(api, "tiny_set_stage_dynamics") ||
    contains(api, "tiny_set_tv_dynamics");
  const bool has_terminal_cost_api =
    contains(api, "tiny_set_terminal_cost") ||
    contains(api, "tiny_set_terminal_matrix");
  if (has_stage_dynamics_api || has_terminal_cost_api) {
    std::cerr << "TinyMPC API changed; refresh the parity decision before use\n";
    return 2;
  }

  TinyWorkspace workspace{};
  if (workspace.Adyn.rows() != 0 || workspace.Bdyn.rows() != 0 ||
      workspace.fdyn.rows() != 0) {
    std::cerr << "unexpected TinyMPC workspace initialization\n";
    return 3;
  }

  std::cout << "TinyMPC parity-gap test passed: one fixed A/B/f model and "
               "no explicit terminal-cost setter in the pinned API\n";
  std::cout << "Migration remains blocked until a maintained fork or an "
               "intentional controller redesign supplies those semantics\n";
  return 0;
}
