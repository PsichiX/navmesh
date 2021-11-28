use crate::Scalar;
use petgraph::{
    algo::{astar, tarjan_scc},
    graph::NodeIndex,
    visit::EdgeRef,
    Directed, Graph,
};
use serde::{de::DeserializeOwned, Deserialize, Serialize};
#[cfg(not(feature = "scalar64"))]
use std::f32::MAX as SCALAR_MAX;
#[cfg(feature = "scalar64")]
use std::f64::MAX as SCALAR_MAX;
use std::{
    collections::{HashMap, HashSet},
    hash::Hash,
};
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

/// Nav islands identifier.
pub type NavIslandsID = ID<NavIslands<(), ()>>;

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct NavIslandPortal<Island, Portal>
where
    Island: std::fmt::Debug + Clone + Eq + Hash + Send + Sync,
    Portal: std::fmt::Debug + Clone + Eq + Hash + Send + Sync,
{
    #[serde(bound(deserialize = "Island: Serialize + DeserializeOwned"))]
    pub island: Island,
    #[serde(bound(deserialize = "Portal: Serialize + DeserializeOwned"))]
    pub portal: Option<Portal>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavIslandsConnection<Island, Portal>
where
    Island: std::fmt::Debug + Clone + Eq + Hash + Send + Sync,
    Portal: std::fmt::Debug + Clone + Eq + Hash + Send + Sync,
{
    #[serde(bound(
        deserialize = "Island: Serialize + DeserializeOwned, Portal: Serialize + DeserializeOwned"
    ))]
    pub from: NavIslandPortal<Island, Portal>,
    #[serde(bound(
        deserialize = "Island: Serialize + DeserializeOwned, Portal: Serialize + DeserializeOwned"
    ))]
    pub to: NavIslandPortal<Island, Portal>,
    pub distance: Scalar,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct NavIslands<Island, Portal>
where
    Island: std::fmt::Debug + Clone + Eq + Hash + Send + Sync,
    Portal: std::fmt::Debug + Clone + Eq + Hash + Send + Sync,
{
    id: NavIslandsID,
    costs: Vec<Scalar>,
    #[serde(bound(
        deserialize = "Island: Serialize + DeserializeOwned, Portal: Serialize + DeserializeOwned"
    ))]
    portals: Vec<NavIslandPortal<Island, Portal>>,
    graph: Graph<(), Scalar, Directed>,
    nodes: Vec<NodeIndex>,
    nodes_map: HashMap<NodeIndex, usize>,
}

impl<Island, Portal> NavIslands<Island, Portal>
where
    Island: std::fmt::Debug + Clone + Eq + Hash + Send + Sync,
    Portal: std::fmt::Debug + Clone + Eq + Hash + Send + Sync,
{
    pub fn new(connections: Vec<NavIslandsConnection<Island, Portal>>, both_ways: bool) -> Self {
        let portals = connections
            .iter()
            .map(|c| c.from.clone())
            .chain(connections.iter().map(|c| c.to.clone()))
            .collect::<HashSet<_>>()
            .into_iter()
            .collect::<Vec<_>>();
        let costs = vec![1.0; portals.len()];
        let mut graph =
            Graph::<(), Scalar, Directed>::with_capacity(portals.len(), connections.len());
        let nodes = (0..portals.len())
            .map(|_| graph.add_node(()))
            .collect::<Vec<_>>();
        for connection in connections {
            let ia = portals.iter().position(|c| &connection.from == c);
            let ib = portals.iter().position(|c| &connection.to == c);
            if let (Some(ia), Some(ib)) = (ia, ib) {
                graph.add_edge(nodes[ia], nodes[ib], connection.distance);
                if both_ways {
                    graph.add_edge(nodes[ib], nodes[ia], connection.distance);
                }
            }
        }
        let nodes_map = iter!(nodes).enumerate().map(|(i, n)| (*n, i)).collect();
        Self {
            id: NavIslandsID::new(),
            costs,
            portals,
            graph,
            nodes,
            nodes_map,
        }
    }

    #[inline]
    pub fn id(&self) -> NavIslandsID {
        self.id
    }

    #[inline]
    pub fn portals(&self) -> &[NavIslandPortal<Island, Portal>] {
        &self.portals
    }

    #[inline]
    pub fn portals_costs(&self) -> &[Scalar] {
        &self.costs
    }

    #[inline]
    pub fn set_portal_cost(
        &mut self,
        portal: &NavIslandPortal<Island, Portal>,
        cost: Scalar,
    ) -> Option<Scalar> {
        let index = self.index(portal)?;
        let c = self.costs.get_mut(index)?;
        let old = *c;
        *c = cost.max(0.0);
        Some(old)
    }

    pub fn neighbors(
        &self,
        portal: &NavIslandPortal<Island, Portal>,
    ) -> Option<impl Iterator<Item = &NavIslandPortal<Island, Portal>> + '_> {
        let index = self.index(portal)?;
        let node = self.nodes[index];
        Some(self.graph.neighbors(node).filter_map(|node| {
            self.nodes_map
                .get(&node)
                .and_then(|index| self.portal(*index))
        }))
    }

    pub fn find_path(
        &self,
        from: &NavIslandPortal<Island, Portal>,
        to: &NavIslandPortal<Island, Portal>,
    ) -> Option<(Scalar, Vec<&NavIslandPortal<Island, Portal>>)> {
        self.find_path_custom(from, to, |_, _| true)
    }

    // filter params: first island-portal, second island-portal.
    pub fn find_path_custom<F>(
        &self,
        from: &NavIslandPortal<Island, Portal>,
        to: &NavIslandPortal<Island, Portal>,
        mut filter: F,
    ) -> Option<(Scalar, Vec<&NavIslandPortal<Island, Portal>>)>
    where
        F: FnMut(&NavIslandPortal<Island, Portal>, &NavIslandPortal<Island, Portal>) -> bool,
    {
        let start_index = self.index(from)?;
        let end_index = self.index(to)?;
        let start_node = *self.nodes.get(start_index)?;
        let end_node = *self.nodes.get(end_index)?;
        let (distance, nodes) = astar(
            &self.graph,
            start_node,
            |n| n == end_node,
            |e| {
                let a = self.nodes_map[&e.source()];
                let b = self.nodes_map[&e.target()];
                let w = *e.weight();
                if filter(self.portal(a).unwrap(), self.portal(b).unwrap()) {
                    let a = self.costs[a];
                    let b = self.costs[b];
                    w * a * b
                } else {
                    SCALAR_MAX
                }
            },
            |_| 0.0,
        )?;
        Some((
            distance,
            nodes
                .into_iter()
                .filter_map(|n| self.portal(self.nodes_map[&n]))
                .collect::<Vec<_>>(),
        ))
    }

    pub fn find_islands(&self) -> Vec<Vec<&NavIslandPortal<Island, Portal>>> {
        tarjan_scc(&self.graph)
            .into_iter()
            .map(|v| {
                v.into_iter()
                    .filter_map(|n| self.nodes_map.get(&n).and_then(|i| self.portal(*i)))
                    .collect::<Vec<_>>()
            })
            .filter(|v| !v.is_empty())
            .collect()
    }

    pub fn index(&self, portal: &NavIslandPortal<Island, Portal>) -> Option<usize> {
        self.portals.iter().position(|p| portal == p)
    }

    pub fn portal(&self, index: usize) -> Option<&NavIslandPortal<Island, Portal>> {
        self.portals.get(index)
    }
}
