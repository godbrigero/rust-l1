# Rust-L1: Unitree LiDAR SDK for Rust

A Rust wrapper for the Unitree LiDAR SDK, providing safe and idiomatic Rust bindings to interface with Unitree L1 LiDAR devices.

## Features

- Safe Rust bindings to the Unitree LiDAR C++ SDK
- Asynchronous data streaming support using Tokio
- Point cloud data processing with nalgebra
- DBSCAN clustering integration
- 3D visualization capabilities with kiss3d

## Installation

### Prerequisites

- Rust toolchain (2021 edition or newer)
- C++ compiler (for binding to native libraries)
- Unitree LiDAR SDK libraries

### Build from Source

```bash
# Clone the repository
git clone https://github.com/yourusername/rust-l1.git
cd rust-l1

# Build the project
./build.sh
```

## Usage

```rust
use rust_l1::lidar_reader::LidarReader;
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // Initialize the LiDAR reader with port, baudrate, and distance thresholds
    let lidar = LidarReader::new_with_initialize("/dev/ttyUSB0", 921600, 30.0, 0.1)?;
    
    // Process point cloud data
    loop {
        match lidar.run_parse() {
            rust_l1::MessageType::POINTCLOUD => {
                let cloud = lidar.get_cloud();
                let points = cloud.points();
                
                println!("Received {} points", points.len());
                // Process points here
            }
            _ => {}
        }
    }
}
```

## Advanced Usage

### Stream Processing with Async/Await

```rust
use futures_util::StreamExt;
use rust_l1::lidar_reader::LidarReader;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let lidar = LidarReader::new_with_initialize("/dev/ttyUSB0", 921600, 30.0, 0.1)?;
    let mut stream = lidar.into_stream();
    
    while let Some(points) = stream.next().await {
        println!("Received {} 3D points", points.len());
        // Process the 3D vectors
    }
    
    Ok(())
}
```

## Project Status

**⚠️ EARLY DEVELOPMENT - CONTRIBUTORS NEEDED! ⚠️**

This project is in early development and needs significant improvement. The current implementation serves as a functional proof of concept, but there are many areas that need work:

- Documentation is sparse
- Error handling is minimal
- Test coverage is insufficient
- API design needs refinement
- Memory safety concerns at FFI boundaries

If you have experience with Rust, LiDAR technologies, or C++ interop, your contributions would be extremely valuable!

## Contributing

Contributions are not just welcome but actively encouraged! Here's how you can help:

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

Even small improvements like better documentation, code cleanup, or bug fixes are highly appreciated.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Unitree Robotics for their LiDAR hardware and SDK
- Rust community for guidance and support