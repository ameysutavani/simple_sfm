## Design considerstions made for the development of this library
1. Keep it simple: The library should be simple to use and understand. Defer design decisions that do not have a clear use case.
1. Use modern C++: Use modern C++ features to make the library more efficient and easier to use.
1. Clean interfaces: Use of third-party libraries should be abstracted behind clean interfaces to make the library more modular and easier to maintain.
1. Extensibility: The library should be easily extensible to support additional features and optimization techniques.
1. Minimize dynamic memory allocation where possible.
1. Extensively document design decisions and assumptions made in the codebase.
1. Test driven development: No one writes perfect code. Create unit tests (even simple ones) to ensure that the code behaves as expected.

## Tools and automation used in the development of this library
1. [Conan](https://conan.io/): Conan is a C/C++ package manager that allows you to manage dependencies for your C/C++ projects. It is used to manage dependencies such as GTSAM and Catch2 in this project.
1. pre-commit: pre-commit is a framework for managing and maintaining multi-language pre-commit hooks. It is used to run code formatting and linting tools such as clang-format and clang-tidy before committing changes to the repository.
1. Interactive debugging with vscode cmake tools: The simple_sfm library project is set up to use the CMake Tools extension in Visual Studio Code for interactive debugging. This allows you to set breakpoints and step through the code while debugging.
