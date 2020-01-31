#[cfg(test)]
#[macro_use]
extern crate approx;

mod nav_mesh;
mod nav_net;
mod nav_vec3;

pub use nav_mesh::*;
pub use nav_net::*;
pub use nav_vec3::*;
use serde::{Deserialize, Serialize};
use std::{
    hash::{Hash, Hasher},
    result::Result as StdResult,
};

pub type Scalar = f64;

/// Error data.
#[derive(Debug, Clone)]
pub enum Error {
    /// Trying to construct triangle with vertice index out of vertices list.
    /// (triangle index, local vertice index, global vertice index)
    TriangleVerticeIndexOutOfBounds(u32, u8, u32),
    /// Trying to construct connection with vertice index out of vertices list.
    /// (connection index, local vertice index, global vertice index)
    ConnectionVerticeIndexOutOfBounds(u32, u8, u32),
    /// Could not serialize NavMesh. Contains serialization error string.
    CouldNotSerializeNavMesh(String),
    /// Could not deserialize NavMesh. Contains deserialization error string.
    CouldNotDeserializeNavMesh(String),
}

/// Result data.
pub type NavResult<T> = StdResult<T, Error>;

#[derive(Debug, Default, Copy, Clone, Eq, Serialize, Deserialize)]
pub struct NavConnection(pub u32, pub u32);

impl Hash for NavConnection {
    fn hash<H: Hasher>(&self, state: &mut H) {
        let first = self.0.min(self.1);
        let second = self.0.max(self.1);
        first.hash(state);
        second.hash(state);
    }
}

impl PartialEq for NavConnection {
    fn eq(&self, other: &Self) -> bool {
        let first = self.0.min(self.1);
        let second = self.0.max(self.1);
        let ofirst = other.0.min(other.1);
        let osecond = other.0.max(other.1);
        first == ofirst && second == osecond
    }
}

pub(crate) const ZERO_TRESHOLD: Scalar = 1e-6;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_raycast() {
        assert_eq!(
            NavVec3::raycast_plane(
                (-1.0, -1.0, -1.0).into(),
                (1.0, 1.0, 1.0).into(),
                (0.0, 0.0, 0.0).into(),
                (-1.0, 0.0, 0.0).into(),
            )
            .unwrap(),
            (0.0, 0.0, 0.0).into(),
        );
        assert_eq!(
            NavVec3::raycast_plane(
                (0.0, 0.0, 0.0).into(),
                (2.0, 1.0, 1.0).into(),
                (1.0, 0.0, 0.0).into(),
                (-1.0, 0.0, 0.0).into(),
            )
            .unwrap(),
            (1.0, 0.5, 0.5).into(),
        );
        assert_eq!(
            NavVec3::raycast_line(
                (-1.0, -1.0, 1.0).into(),
                (1.0, 1.0, 1.0).into(),
                (1.0, -1.0, 0.0).into(),
                (-1.0, 1.0, 0.0).into(),
                (-1.0, 0.0, 0.0).into(),
            )
            .unwrap(),
            (0.0, 0.0, 0.0).into(),
        );
        assert_eq!(
            NavVec3::raycast_line(
                (0.0, 0.0, 0.0).into(),
                (2.0, 1.0, 1.0).into(),
                (1.0, 0.0, 0.0).into(),
                (1.0, 1.0, 0.0).into(),
                (1.0, 0.0, 0.0).into(),
            )
            .unwrap(),
            (1.0, 0.5, 0.0).into(),
        );
        assert_eq!(
            NavVec3::raycast_triangle(
                (0.0, 0.0, 1.0).into(),
                (0.0, 0.0, -1.0).into(),
                (0.0, -1.0, 0.0).into(),
                (1.0, 1.0, 0.0).into(),
                (-1.0, 1.0, 0.0).into(),
            )
            .unwrap(),
            (0.0, 0.0, 0.0).into(),
        );
        assert_eq!(
            NavVec3::raycast_triangle(
                (-1.0, -1.0, 1.0).into(),
                (-1.0, -1.0, -1.0).into(),
                (0.0, -1.0, 0.0).into(),
                (1.0, 1.0, 0.0).into(),
                (-1.0, 1.0, 0.0).into(),
            ),
            None,
        );
    }

    #[test]
    fn test_line_between_points() {
        assert_eq!(
            true,
            NavVec3::is_line_between_points(
                (0.0, -1.0, 0.0).into(),
                (0.0, 1.0, 0.0).into(),
                (-1.0, 0.0, 0.0).into(),
                (1.0, 0.0, 0.0).into(),
                (0.0, 0.0, 1.0).into(),
            ),
        );
        assert_eq!(
            false,
            NavVec3::is_line_between_points(
                (-2.0, -1.0, 0.0).into(),
                (-2.0, 1.0, 0.0).into(),
                (-1.0, 0.0, 0.0).into(),
                (1.0, 0.0, 0.0).into(),
                (0.0, 0.0, 1.0).into(),
            ),
        );
        assert_eq!(
            false,
            NavVec3::is_line_between_points(
                (2.0, -1.0, 0.0).into(),
                (2.0, 1.0, 0.0).into(),
                (-1.0, 0.0, 0.0).into(),
                (1.0, 0.0, 0.0).into(),
                (0.0, 0.0, 1.0).into(),
            ),
        );
        assert_eq!(
            true,
            NavVec3::is_line_between_points(
                (-1.0, -1.0, 0.0).into(),
                (-1.0, 1.0, 0.0).into(),
                (-1.0, 0.0, 0.0).into(),
                (1.0, 0.0, 0.0).into(),
                (0.0, 0.0, 1.0).into(),
            ),
        );
        assert_eq!(
            true,
            NavVec3::is_line_between_points(
                (1.0, -1.0, 0.0).into(),
                (1.0, 1.0, 0.0).into(),
                (-1.0, 0.0, 0.0).into(),
                (1.0, 0.0, 0.0).into(),
                (0.0, 0.0, 1.0).into(),
            ),
        );
    }

    #[test]
    fn test_spatials() {
        {
            let vertices = vec![
                (0.0, 0.0, 0.0).into(),
                (2.0, 0.0, 0.0).into(),
                (0.0, 2.0, 0.0).into(),
            ];

            let s = NavSpatialObject::new(0, vertices[0], vertices[1], vertices[2]);
            assert_eq!(s.normal(), (0.0, 0.0, 1.0).into());
        }
        {
            let vertices = vec![
                (0.0, 0.0, 0.0).into(),
                (2.0, 0.0, 2.0).into(),
                (0.0, 2.0, 0.0).into(),
            ];

            let s = NavSpatialObject::new(0, vertices[0], vertices[1], vertices[2]);
            assert_eq!(s.normal(), NavVec3::new(-1.0, 0.0, 1.0).normalize());
        }
        {
            let vertices = vec![
                (1.0, 2.0, 0.0).into(),
                (2.0, 2.0, 0.0).into(),
                (2.0, 3.0, 0.0).into(),
                (1.0, 3.0, 0.0).into(),
            ];

            let s = NavSpatialObject::new(0, vertices[0], vertices[1], vertices[2]);
            assert_eq!(s.closest_point(vertices[0]), vertices[0]);
            assert_eq!(s.closest_point(vertices[1]), vertices[1]);
            assert_eq!(s.closest_point(vertices[2]), vertices[2]);
            assert_eq!(
                s.closest_point((1.75, 2.25, 0.0).into()),
                (1.75, 2.25, 0.0).into()
            );
            assert_eq!(
                s.closest_point((1.5, 1.0, 0.0).into()),
                (1.5, 2.0, 0.0).into()
            );
            assert_eq!(
                s.closest_point((3.0, 2.5, 0.0).into()),
                (2.0, 2.5, 0.0).into()
            );
            assert_eq!(
                s.closest_point((1.0, 3.0, 0.0).into()),
                (1.5, 2.5, 0.0).into()
            );

            let s = NavSpatialObject::new(0, vertices[2], vertices[3], vertices[0]);
            assert_eq!(s.closest_point(vertices[2]), vertices[2]);
            assert_eq!(s.closest_point(vertices[3]), vertices[3]);
            assert_eq!(s.closest_point(vertices[0]), vertices[0]);
            assert_eq!(
                s.closest_point((1.25, 2.75, 0.0).into()),
                (1.25, 2.75, 0.0).into()
            );
            assert_eq!(
                s.closest_point((2.0, 2.0, 0.0).into()),
                (1.5, 2.5, 0.0).into()
            );
            assert_eq!(
                s.closest_point((1.5, 4.0, 0.0).into()),
                (1.5, 3.0, 0.0).into()
            );
            assert_eq!(
                s.closest_point((0.0, 2.5, 0.0).into()),
                (1.0, 2.5, 0.0).into()
            );
        }
    }

    #[test]
    fn test_general() {
        let vertices = vec![
            (0.0, 0.0, 0.0).into(), // 0
            (1.0, 0.0, 0.0).into(), // 1
            (2.0, 0.0, 0.0).into(), // 2
            (0.0, 1.0, 0.0).into(), // 3
            (1.0, 1.0, 0.0).into(), // 4
            (2.0, 1.0, 0.0).into(), // 5
            (0.0, 2.0, 0.0).into(), // 6
            (1.0, 2.0, 0.0).into(), // 7
        ];
        let triangles = vec![
            (0, 1, 4).into(), // 0
            (4, 3, 0).into(), // 1
            (1, 2, 5).into(), // 2
            (5, 4, 1).into(), // 3
            (3, 4, 7).into(), // 4
            (7, 6, 3).into(), // 5
        ];
        let mesh = NavMesh::new(vertices.clone(), triangles.clone()).unwrap();
        {
            let path = mesh.find_path_triangles(0, 0).unwrap().0;
            assert_eq!(path, vec![0]);
        }
        {
            let path = mesh.find_path_triangles(2, 5).unwrap().0;
            assert_eq!(path, vec![2, 3, 0, 1, 4, 5]);
        }
        {
            let path = mesh
                .find_path(
                    (0.0, 0.0, 0.0).into(),
                    (2.0, 0.0, 0.0).into(),
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
                vec![(0, 0, 0), (20, 0, 0),]
            );
            let path = mesh
                .find_path(
                    (0.0, 0.0, 0.0).into(),
                    (2.0, 0.0, 0.0).into(),
                    NavQuery::Accuracy,
                    NavPathMode::Accuracy,
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
                vec![(0, 0, 0), (20, 0, 0),]
            );
        }
        {
            let path = mesh
                .find_path(
                    (2.0, 0.0, 0.0).into(),
                    (0.0, 2.0, 0.0).into(),
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
                vec![(20, 0, 0), (0, 20, 0),]
            );
            let path = mesh
                .find_path(
                    (2.0, 0.0, 0.0).into(),
                    (0.0, 2.0, 0.0).into(),
                    NavQuery::Accuracy,
                    NavPathMode::Accuracy,
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
                vec![(20, 0, 0), (10, 10, 0), (0, 20, 0),]
            );
        }
        {
            let path = mesh
                .find_path(
                    (2.0, 1.0, 0.0).into(),
                    (1.0, 2.0, 0.0).into(),
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
                vec![(20, 10, 0), (5, 10, 0), (10, 20, 0),]
            );
            let path = mesh
                .find_path(
                    (2.0, 1.0, 0.0).into(),
                    (1.0, 2.0, 0.0).into(),
                    NavQuery::Accuracy,
                    NavPathMode::Accuracy,
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
                vec![(20, 10, 0), (10, 10, 0), (10, 20, 0),]
            );
        }
        {
            let path = mesh
                .find_path(
                    (0.5, 0.0, 0.0).into(),
                    (0.5, 2.0, 0.0).into(),
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
                vec![(5, 0, 0), (5, 20, 0),]
            );
            let path = mesh
                .find_path(
                    (0.5, 0.0, 0.0).into(),
                    (0.5, 2.0, 0.0).into(),
                    NavQuery::Accuracy,
                    NavPathMode::Accuracy,
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
                vec![(5, 0, 0), (5, 20, 0),]
            );
        }

        let vertices = vec![
            (0.0, 0.0, 0.0).into(), // 0
            (2.0, 0.0, 0.0).into(), // 1
            (2.0, 1.0, 0.0).into(), // 2
            (1.0, 1.0, 0.0).into(), // 3
            (0.0, 2.0, 0.0).into(), // 4
        ];
        let triangles = vec![
            (0, 3, 4).into(), // 0
            (0, 1, 3).into(), // 1
            (1, 2, 3).into(), // 2
        ];
        let mesh = NavMesh::new(vertices.clone(), triangles.clone()).unwrap();
        {
            let path = mesh.find_path_triangles(0, 2).unwrap().0;
            assert_eq!(path, vec![0, 1, 2]);
        }
        {
            let path = mesh
                .find_path(
                    (2.0, 1.0, 0.0).into(),
                    (0.0, 2.0, 0.0).into(),
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
                vec![(20, 10, 0), (5, 5, 0), (0, 20, 0),]
            );
            let path = mesh
                .find_path(
                    (2.0, 1.0, 0.0).into(),
                    (0.0, 2.0, 0.0).into(),
                    NavQuery::Accuracy,
                    NavPathMode::Accuracy,
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
                vec![(20, 10, 0), (10, 10, 0), (0, 20, 0),]
            );
        }

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
        let mesh = NavMesh::new(vertices.clone(), triangles.clone()).unwrap();
        {
            let path = mesh.find_path_triangles(1, 2).unwrap().0;
            assert_eq!(path, vec![1, 0, 3, 2]);
        }
        {
            let path = mesh
                .find_path(
                    (0.0, 0.5, 0.0).into(),
                    (2.0, 0.5, 1.0).into(),
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
                vec![(0, 5, 0), (10, 5, 0), (20, 5, 10),]
            );
            let path = mesh
                .find_path(
                    (0.0, 0.5, 0.0).into(),
                    (2.0, 0.5, 1.0).into(),
                    NavQuery::Accuracy,
                    NavPathMode::Accuracy,
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
                vec![(0, 5, 0), (10, 5, 0), (20, 5, 10),]
            );
        }
        {
            let path = mesh
                .find_path(
                    (0.0, 1.0, 0.0).into(),
                    (2.0, 0.0, 1.0).into(),
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
                vec![(0, 10, 0), (10, 5, 0), (20, 0, 10),]
            );
            let path = mesh
                .find_path(
                    (0.0, 1.0, 0.0).into(),
                    (2.0, 0.0, 1.0).into(),
                    NavQuery::Accuracy,
                    NavPathMode::Accuracy,
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
                vec![(0, 10, 0), (10, 5, 0), (20, 0, 10),]
            );
        }
        {
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
            let path = mesh
                .find_path(
                    (0.0, 1.0, 0.0).into(),
                    (1.2, 0.4, 0.2).into(),
                    NavQuery::Accuracy,
                    NavPathMode::Accuracy,
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
                vec![(0, 10, 0), (10, 5, 0), (12, 4, 2),]
            );
        }
    }

    #[test]
    fn test_thicken() {
        let source = NavMesh::new(
            vec![
                [-10.0, -10.0, 0.0].into(),
                [10.0, -10.0, 0.0].into(),
                [10.0, 10.0, 0.0].into(),
                [-10.0, 10.0, 0.0].into(),
            ],
            vec![[0, 1, 2].into(), [2, 3, 0].into()],
        )
        .unwrap();
        let thickened = source.thicken(1.0).unwrap();
        for (a, b) in source.vertices().iter().zip(thickened.vertices().iter()) {
            assert_relative_eq!(*b, *a + NavVec3::new(0.0, 0.0, 1.0));
        }

        let source = NavMesh::new(
            vec![
                [-5.0, -5.0, -5.0].into(), // 0
                [5.0, -5.0, -5.0].into(),  // 1
                [5.0, 5.0, -5.0].into(),   // 2
                [-5.0, 5.0, -5.0].into(),  // 3
                [-5.0, -5.0, 5.0].into(),  // 4
                [5.0, -5.0, 5.0].into(),   // 5
                [5.0, 5.0, 5.0].into(),    // 6
                [-5.0, 5.0, 5.0].into(),   // 7
            ],
            vec![
                [2, 1, 0].into(),
                [0, 3, 2].into(),
                [4, 5, 6].into(),
                [6, 7, 4].into(),
                [0, 1, 5].into(),
                [5, 4, 0].into(),
                [1, 2, 6].into(),
                [6, 5, 1].into(),
                [2, 3, 7].into(),
                [7, 6, 2].into(),
                [3, 0, 4].into(),
                [4, 7, 3].into(),
            ],
        )
        .unwrap();
        let thickened = source.thicken(1.0).unwrap();
        let expected = vec![
            NavVec3 {
                x: -5.333333333333333,
                y: -5.666666666666667,
                z: -5.666666666666667,
            },
            NavVec3 {
                x: 5.816496580927726,
                y: -5.408248290463863,
                z: -5.408248290463863,
            },
            NavVec3 {
                x: 5.333333333333333,
                y: 5.666666666666667,
                z: -5.666666666666667,
            },
            NavVec3 {
                x: -5.816496580927726,
                y: 5.408248290463863,
                z: -5.408248290463863,
            },
            NavVec3 {
                x: -5.666666666666667,
                y: -5.333333333333333,
                z: 5.666666666666667,
            },
            NavVec3 {
                x: 5.408248290463863,
                y: -5.816496580927726,
                z: 5.408248290463863,
            },
            NavVec3 {
                x: 5.666666666666667,
                y: 5.333333333333333,
                z: 5.666666666666667,
            },
            NavVec3 {
                x: -5.408248290463863,
                y: 5.816496580927726,
                z: 5.408248290463863,
            },
        ];
        for (a, b) in expected.iter().zip(thickened.vertices().iter()) {
            assert_relative_eq!(a, b);
        }
    }
}
