# Github workflows

This directory contains the workflow scripts for the 'swerve_vehicle_descriptors' package. The different workflows are:

- [ci](ci.yml): This workflow runs the unit tests and calculates unit test coverage for the package.
- [docs](docs.yml): This workflow generates the documentation for the package.
- [find_mutants_in_pr](find_mutants_in_pr.yml): This workflow runs the mutation testing tool [cargo mutants](https://mutants.rs/)
  on the changes in the current pull request.
- [lint_pr](lint_pr.yml): This workflow runs the linter on the current pull request and checks that the
  pull request title aligns with the [conventional commits](https://www.conventionalcommits.org/en/v1.0.0/) format.
- [lint](lint.yml): This workflow runs [rustfmt](https://github.com/rust-lang/rustfmt) and [clippy](https://doc.rust-lang.org/clippy/)
  on the source code.
- [release-plz](release-plz.yml): This workflow runs [release-plz](https://release-plz.ieni.dev/) to create a release
  PR and subsequently release the package to [crates.io](https://crates.io/).
