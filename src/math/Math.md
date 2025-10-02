# Math library strategy

We are currently using the math library from Hexops Mach as our foundation because it is stable and well-tested:
- Repository: https://github.com/hexops/mach

We will build upon it based on necessity, extending or adapting functionality as the project evolves.

## Integration status

- The math module is exposed as a Zig build module named "math" and imported by the executable.
- Modifications were made to remove mach's math dependencies to other modules
- See build.zig for the module wiring:
  - math module root: src/math/math.zig
  - Executable imports the "math" module
  - Tests are run against the math module

## Future direction

- Extend functionality incrementally within src/math as needed.
