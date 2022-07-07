// swift-tools-version: 5.6
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

let package = Package(
    name: "box2d",
    products: [
        .executable(
            name: "box2d-test",
            targets: ["box2d-test"]
        ),
        .library(
            name: "box2d",
            targets: ["box2d"]),
    ],
    targets: [
        .executableTarget(
            name: "box2d-test",
            dependencies: ["box2d"]
        ),
        .target(
            name: "box2d",
            sources: ["src"],
            cxxSettings: [
                .headerSearchPath("include/box2d"),
                .headerSearchPath("src/dynamics")
            ]
        )
    ],
    cxxLanguageStandard: .cxx11
)
