# 2D Physics Engine

This is a 2D Physics Engine created using C++ and SFML. The main goal was to get better C++ development skills and learn a little CMake. The project includes a demo that can be found in `main.cpp`.

## Getting Started

### Prerequisites

- C++ compiler (e.g., g++)
- CMake
- SFML library (version 2.5.1 or higher, but less than 3.0.0)

### Building the Project

1. Clone the repository:
    ```sh
    git clone https://github.com/lorenzo-rod/2DPhysicsEngine.git
    cd 2DPhysicsEngine
    ```

2. Create a build directory and navigate into it:
    ```sh
    mkdir build
    cd build
    ```

3. Run CMake to configure the project in release mode:
    ```sh
    cmake -DCMAKE_BUILD_TYPE=Release ..
    ```

4. Build the project:
    ```sh
    make
    ```

### Running the Demo

After building the project, you can run the demo:
```sh
./2DPhysicsEngineDemo
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
