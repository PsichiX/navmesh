# List the just recipe list
list:
    just --list

# Mandatory checks to run before pushing changes to repository
checks:
    cargo fmt
    cargo build
    cargo clippy
    cargo test
