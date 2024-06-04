// swift-tools-version: 5.9
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

let package = Package(
    name: "box2d",
    platforms: [
        .macOS(.v14)
    ],
    products: [
        .library(
            name: "box2d",
            targets: ["box2d"]
        )
    ],
    targets: [
        .target(
            name: "box2d",
            swiftSettings: [
                .interoperabilityMode(.Cxx)
            ]
        )
    ],
    cxxLanguageStandard: .cxx20
)
