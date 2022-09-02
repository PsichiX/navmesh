# NavMesh ![travis-ci status](https://travis-ci.org/PsichiX/navmesh.svg?branch=master) ![crates-io version](https://raster.shields.io/crates/v/navmesh.png)
#### Nav-Mesh path finder for Rust

## Installation
Cargo.toml
```toml
[dependencies]
navmesh = "0.8"
```

## Example
```rust
use navmesh::*;

let vertices = vec![
    (0.0, 0.0, 0.0).into(), // 0
    (1.0, 0.0, 0.0).into(), // 1
    (2.0, 0.0, 1.0).into(), // 2
    (0.0, 1.0, 0.0).into(), // 3
    (1.0, 1.0, 0.0).into(), // 4
    (2.0, 1.0, 1.0).into(), // 5
];
let triangles = vec![
    (0, 1, 4).into(), // 0
    (4, 3, 0).into(), // 1
    (1, 2, 5).into(), // 2
    (5, 4, 1).into(), // 3
];

let mesh = NavMesh::new(vertices, triangles).unwrap();
let path = mesh
    .find_path(
        (0.0, 1.0, 0.0).into(),
        (1.5, 0.25, 0.5).into(),
        NavQuery::Accuracy,
        NavPathMode::MidPoints,
    )
    .unwrap();
assert_eq!(
    path.into_iter()
        .map(|v| (
            (v.x * 10.0) as i32,
            (v.y * 10.0) as i32,
            (v.z * 10.0) as i32,
        ))
        .collect::<Vec<_>>(),
    vec![(0, 10, 0), (10, 5, 0), (15, 2, 5),]
);
```

## Web assembly
To use Web Assembly support enable the `web` feature flag.
