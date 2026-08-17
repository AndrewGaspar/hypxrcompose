#pragma once

// `hypxrcompose validate <take>`.
//
// This command is the format's arbiter: it runs the same loader the compositor
// runs, with media probing on, and prints every structural and referential
// complaint rather than stopping at the first. A bundle that validates clean is
// one `render` can consume; a bundle that only warns is one `render` will
// consume with the stated caveats.

#include <filesystem>
#include <string>

namespace hxc {

    struct SValidateOptions {
        std::filesystem::path root;
        // Promote warnings to errors, for a producer's own CI.
        bool                  strict     = false;
        bool                  jsonOutput = false;
        // Skips ffprobe. Structure only, for a quick syntax check.
        bool                  skipMedia  = false;
    };

    // 0 = clean (or warnings only, unless --strict), 1 = errors, 2 = the bundle
    // could not be opened at all.
    int runValidate(const SValidateOptions& options);

    // Human-readable one-paragraph summary of what a loaded bundle holds. Shared
    // with `render`, which prints it before composing.
    std::string describeBundle(const struct SBundle& bundle);

}
