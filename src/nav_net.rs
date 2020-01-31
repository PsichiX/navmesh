use crate::{Error, NavConnection, NavResult, NavVec3, Scalar};
use petgraph::{algo::astar, graph::NodeIndex, visit::EdgeRef, Graph, Undirected};
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use spade::{rtree::RTree, BoundingRect, SpatialObject};
use std::collections::HashMap;
use typid::ID;

#[cfg(feature = "parallel")]
macro_rules! iter {
    ($v:expr) => {
        $v.par_iter()
    };
}
#[cfg(not(feature = "parallel"))]
macro_rules! iter {
    ($v:expr) => {
        $v.iter()
    };
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavSpatialConnection {
    pub connection: NavConnection,
    pub index: usize,
    pub a: NavVec3,
    pub b: NavVec3,
}

impl NavSpatialConnection {
    pub fn new(connection: NavConnection, index: usize, a: NavVec3, b: NavVec3) -> Self {
        Self {
            connection,
            index,
            a,
            b,
        }
    }

    pub fn closest_point(&self, point: NavVec3) -> NavVec3 {
        let t = point.project(self.a, self.b);
        NavVec3::unproject(self.a, self.b, t)
    }
}

impl SpatialObject for NavSpatialConnection {
    type Point = NavVec3;

    fn mbr(&self) -> BoundingRect<Self::Point> {
        let min = NavVec3::new(
            self.a.x.min(self.b.x),
            self.a.y.min(self.b.y),
            self.a.z.min(self.b.z),
        );
        let max = NavVec3::new(
            self.a.x.max(self.b.x),
            self.a.y.max(self.b.y),
            self.a.z.max(self.b.z),
        );
        BoundingRect::from_corners(&min, &max)
    }

    fn distance2(&self, point: &Self::Point) -> Scalar {
        (*point - self.closest_point(*point)).sqr_magnitude()
    }
}

/// Nav net identifier.
pub type NavNetID = ID<NavNet>;

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct NavNet {
    id: NavNetID,
    vertices: Vec<NavVec3>,
    connections: Vec<NavConnection>,
    distances: Vec<Scalar>,
    costs: Vec<Scalar>,
    graph: Graph<(), Scalar, Undirected>,
    nodes: Vec<NodeIndex>,
    nodes_map: HashMap<NodeIndex, usize>,
    rtree: RTree<NavSpatialConnection>,
    spatials: Vec<NavSpatialConnection>,
    origin: NavVec3,
}

impl NavNet {
    pub fn new(vertices: Vec<NavVec3>, connections: Vec<NavConnection>) -> NavResult<Self> {
        let origin =
            iter!(vertices).fold(NavVec3::default(), |a, v| a + *v) / vertices.len() as Scalar;

        let distances = iter!(connections)
            .enumerate()
            .map(|(i, c)| {
                if c.0 as usize >= vertices.len() {
                    return Err(Error::ConnectionVerticeIndexOutOfBounds(i as u32, 0, c.0));
                }
                if c.1 as usize >= vertices.len() {
                    return Err(Error::ConnectionVerticeIndexOutOfBounds(i as u32, 1, c.1));
                }
                let a = vertices[c.0 as usize];
                let b = vertices[c.1 as usize];
                Ok((b - a).sqr_magnitude())
            })
            .collect::<NavResult<Vec<_>>>()?;

        let costs = vec![1.0; vertices.len()];

        let mut graph = Graph::<(), Scalar, Undirected>::new_undirected();
        let nodes = (0..vertices.len())
            .map(|_| graph.add_node(()))
            .collect::<Vec<_>>();
        graph.extend_with_edges(
            iter!(connections)
                .enumerate()
                .map(|(i, conn)| (nodes[conn.0 as usize], nodes[conn.1 as usize], distances[i])),
        );
        let nodes_map = iter!(nodes).enumerate().map(|(i, n)| (*n, i)).collect();

        let spatials = iter!(connections)
            .enumerate()
            .map(|(i, connection)| {
                NavSpatialConnection::new(
                    *connection,
                    i,
                    vertices[connection.0 as usize],
                    vertices[connection.1 as usize],
                )
            })
            .collect::<Vec<_>>();

        let mut rtree = RTree::new();
        for spatial in &spatials {
            rtree.insert(spatial.clone());
        }

        Ok(Self {
            id: ID::default(),
            vertices,
            connections,
            distances,
            costs,
            graph,
            nodes,
            nodes_map,
            rtree,
            spatials,
            origin,
        })
    }

    pub fn scale(&self, value: NavVec3, origin: Option<NavVec3>) -> NavResult<Self> {
        let origin = origin.unwrap_or(self.origin);
        let vertices = iter!(self.vertices)
            .map(|v| (*v - origin) * value + origin)
            .collect::<Vec<_>>();
        Self::new(vertices, self.connections.clone())
    }

    #[inline]
    pub fn id(&self) -> NavNetID {
        self.id
    }

    #[inline]
    pub fn origin(&self) -> NavVec3 {
        self.origin
    }

    #[inline]
    pub fn vertices(&self) -> &[NavVec3] {
        &self.vertices
    }

    #[inline]
    pub fn connections(&self) -> &[NavConnection] {
        &self.connections
    }

    #[inline]
    pub fn distances(&self) -> &[Scalar] {
        &self.distances
    }

    #[inline]
    pub fn vertices_costs(&self) -> &[Scalar] {
        &self.costs
    }

    #[inline]
    pub fn set_vertice_cost(&mut self, index: usize, cost: Scalar) -> Option<Scalar> {
        let c = self.costs.get_mut(index)?;
        let old = *c;
        *c = cost.max(0.0);
        Some(old)
    }

    pub fn closest_point(&self, point: NavVec3) -> Option<NavVec3> {
        let index = self.find_closest_connection(point)?;
        Some(self.spatials[index].closest_point(point))
    }

    pub fn find_closest_connection(&self, point: NavVec3) -> Option<usize> {
        self.rtree.nearest_neighbor(&point).map(|c| c.index)
    }

    pub fn find_path(&self, from: NavVec3, to: NavVec3) -> Option<Vec<NavVec3>> {
        let start_index = self.find_closest_connection(from)?;
        let end_index = self.find_closest_connection(to)?;
        let start_connection = self.connections[start_index];
        let end_connection = self.connections[end_index];
        let start_point = self.spatials[start_index].closest_point(from);
        let end_point = self.spatials[end_index].closest_point(to);
        if start_index == end_index {
            return Some(vec![start_point, end_point]);
        } else if start_point.same_as(end_point) {
            return Some(vec![start_point]);
        }
        let start_vertice = {
            let a = self.vertices[start_connection.0 as usize];
            let b = self.vertices[start_connection.1 as usize];
            if (a - start_point).sqr_magnitude() < (b - start_point).sqr_magnitude() {
                start_connection.0 as usize
            } else {
                start_connection.1 as usize
            }
        };
        let end_vertice = {
            let a = self.vertices[end_connection.0 as usize];
            let b = self.vertices[end_connection.1 as usize];
            if (a - end_point).sqr_magnitude() < (b - end_point).sqr_magnitude() {
                end_connection.0 as usize
            } else {
                end_connection.1 as usize
            }
        };
        let start_node = *self.nodes.get(start_vertice)?;
        let end_node = *self.nodes.get(end_vertice)?;
        let nodes = astar(
            &self.graph,
            start_node,
            |n| n == end_node,
            |e| {
                let a = self.costs[self.nodes_map[&e.source()]];
                let b = self.costs[self.nodes_map[&e.target()]];
                *e.weight() * a * b
            },
            |_| 0.0,
        )?
        .1;
        let mut points = nodes
            .into_iter()
            .map(|n| self.vertices[self.nodes_map[&n]])
            .collect::<Vec<_>>();
        if points.len() > 2 {
            {
                let mut iter = points.iter();
                let a = *iter.next()?;
                let b = *iter.next()?;
                let t = start_point.project(a, b);
                if t >= 0.0 && t <= 1.0 {
                    points[0] = start_point;
                } else {
                    points.insert(0, start_point);
                }
            }
            {
                let mut iter = points.iter().rev();
                let a = *iter.next()?;
                let b = *iter.next()?;
                let t = end_point.project(a, b);
                if t >= 0.0 && t <= 1.0 {
                    *points.last_mut()? = end_point;
                } else {
                    points.push(end_point);
                }
            }
        }
        Some(points)
    }
}
