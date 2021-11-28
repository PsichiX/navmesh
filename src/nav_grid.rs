use crate::{Error, NavResult, Scalar};
use petgraph::{
    algo::{astar, tarjan_scc},
    graph::NodeIndex,
    visit::EdgeRef,
    Directed, Graph, Undirected,
};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
#[cfg(not(feature = "scalar64"))]
use std::f32::MAX as SCALAR_MAX;
#[cfg(feature = "scalar64")]
use std::f64::MAX as SCALAR_MAX;
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

#[derive(Debug, Default, Copy, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NavGridConnection {
    pub from: (usize, usize),
    pub to: (usize, usize),
}

/// Nav grid identifier.
pub type NavGridID = ID<NavGrid>;

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct NavGrid {
    id: NavGridID,
    cols: usize,
    rows: usize,
    cells: Vec<bool>,
    costs: Vec<Scalar>,
    graph: Graph<(), (), Directed>,
    nodes: Vec<Option<NodeIndex>>,
    nodes_map: HashMap<NodeIndex, usize>,
}

impl NavGrid {
    pub fn new(cols: usize, rows: usize, cells: Vec<bool>) -> NavResult<Self> {
        if cols == 0 || rows == 0 {
            return Err(Error::EmptyCells(cols, rows));
        }
        if cols * rows != cells.len() {
            return Err(Error::CellsCountDoesNotMatchColsRows(
                cells.len(),
                cols,
                rows,
            ));
        }
        let costs = vec![1.0; cells.len()];
        let mut graph = Graph::<(), (), Directed>::with_capacity(
            cells.len(),
            (cols - 1) * rows + (rows - 1) * cols,
        );
        let nodes = (0..cells.len())
            .zip(cells.iter())
            .map(|(_, cell)| {
                if *cell {
                    Some(graph.add_node(()))
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        for ca in 0..(cols - 1) {
            for r in 0..rows {
                let cb = ca + 1;
                let ia = r * cols + ca;
                let ib = r * cols + cb;
                if let (Some(na), Some(nb)) = (nodes[ia], nodes[ib]) {
                    graph.add_edge(na, nb, ());
                    graph.add_edge(nb, na, ());
                }
            }
        }
        for c in 0..cols {
            for ra in 0..(rows - 1) {
                let rb = ra + 1;
                let ia = ra * cols + c;
                let ib = rb * cols + c;
                if let (Some(na), Some(nb)) = (nodes[ia], nodes[ib]) {
                    graph.add_edge(na, nb, ());
                    graph.add_edge(nb, na, ());
                }
            }
        }
        let nodes_map = iter!(nodes)
            .enumerate()
            .filter_map(|(i, n)| n.map(|n| (n, i)))
            .collect();
        Ok(Self {
            id: NavGridID::new(),
            cols,
            rows,
            cells,
            costs,
            graph,
            nodes,
            nodes_map,
        })
    }

    pub fn with_connections(
        cols: usize,
        rows: usize,
        connections: Vec<NavGridConnection>,
    ) -> NavResult<Self> {
        if cols == 0 || rows == 0 {
            return Err(Error::EmptyCells(cols, rows));
        }
        let count = cols * rows;
        for connection in &connections {
            if connection.from.0 >= cols || connection.from.1 >= rows {
                return Err(Error::InvalidCellCoordinate(
                    connection.from.0,
                    connection.from.1,
                    cols,
                    rows,
                ));
            }
            if connection.to.0 >= cols || connection.to.1 >= rows {
                return Err(Error::InvalidCellCoordinate(
                    connection.to.0,
                    connection.to.1,
                    cols,
                    rows,
                ));
            }
        }
        let costs = vec![1.0; count];
        let mut graph =
            Graph::<(), (), Directed>::with_capacity(count, (cols - 1) * rows + (rows - 1) * cols);
        let nodes = (0..count)
            .map(|index| {
                let coord = (index % cols, index / cols);
                if coord.0 >= cols
                    && coord.1 >= rows
                    && connections.iter().any(|c| c.from == coord || c.to == coord)
                {
                    return None;
                }
                Some(graph.add_node(()))
            })
            .collect::<Vec<_>>();
        for connection in connections {
            let ia = connection.from.1 * cols + connection.from.0;
            let ib = connection.to.1 * cols + connection.to.0;
            if let (Some(na), Some(nb)) = (nodes[ia], nodes[ib]) {
                graph.add_edge(na, nb, ());
            }
        }
        let nodes_map = iter!(nodes)
            .enumerate()
            .filter_map(|(i, n)| n.map(|n| (n, i)))
            .collect();
        Ok(Self {
            id: NavGridID::new(),
            cols,
            rows,
            cells: nodes.iter().map(Option::is_some).collect(),
            costs,
            graph,
            nodes,
            nodes_map,
        })
    }

    #[inline]
    pub fn id(&self) -> NavGridID {
        self.id
    }

    #[inline]
    pub fn cells(&self) -> &[bool] {
        &self.cells
    }

    #[inline]
    pub fn cells_costs(&self) -> &[Scalar] {
        &self.costs
    }

    #[inline]
    pub fn set_cell_cost(&mut self, col: usize, row: usize, cost: Scalar) -> Option<Scalar> {
        let index = self.index(col, row)?;
        let c = self.costs.get_mut(index)?;
        let old = *c;
        *c = cost.max(0.0);
        Some(old)
    }

    pub fn neighbors(
        &self,
        col: usize,
        row: usize,
    ) -> Option<impl Iterator<Item = (usize, usize)> + '_> {
        let index = self.index(col, row)?;
        let node = self.nodes[index]?;
        Some(self.graph.neighbors(node).filter_map(|node| {
            self.nodes_map
                .get(&node)
                .and_then(|index| self.coord(*index))
        }))
    }

    pub fn find_path(
        &self,
        from: (usize, usize),
        to: (usize, usize),
    ) -> Option<Vec<(usize, usize)>> {
        self.find_path_custom(from, to, |_, _| true)
    }

    // filter params: first col-row, second col-row.
    pub fn find_path_custom<F>(
        &self,
        from: (usize, usize),
        to: (usize, usize),
        mut filter: F,
    ) -> Option<Vec<(usize, usize)>>
    where
        F: FnMut((usize, usize), (usize, usize)) -> bool,
    {
        let start_index = self.index(from.0, from.1)?;
        let end_index = self.index(to.0, to.1)?;
        let start_node = (*self.nodes.get(start_index)?)?;
        let end_node = (*self.nodes.get(end_index)?)?;
        let nodes = astar(
            &self.graph,
            start_node,
            |n| n == end_node,
            |e| {
                let a = self.nodes_map[&e.source()];
                let b = self.nodes_map[&e.target()];
                if filter(self.coord(a).unwrap(), self.coord(b).unwrap()) {
                    let a = self.costs[a];
                    let b = self.costs[b];
                    a * b
                } else {
                    SCALAR_MAX
                }
            },
            |_| 0.0,
        )?
        .1;
        Some(
            nodes
                .into_iter()
                .filter_map(|n| self.coord(self.nodes_map[&n]))
                .collect::<Vec<_>>(),
        )
    }

    pub fn find_islands(&self) -> Vec<Vec<(usize, usize)>> {
        tarjan_scc(&self.graph)
            .into_iter()
            .map(|v| {
                v.into_iter()
                    .filter_map(|n| self.nodes_map.get(&n).and_then(|i| self.coord(*i)))
                    .collect::<Vec<_>>()
            })
            .filter(|v| !v.is_empty())
            .collect()
    }

    pub fn index(&self, col: usize, row: usize) -> Option<usize> {
        if col < self.cols && row < self.rows {
            Some(row * self.cols + col)
        } else {
            None
        }
    }

    pub fn coord(&self, index: usize) -> Option<(usize, usize)> {
        let col = index % self.cols;
        let row = index / self.cols;
        if col < self.cols && row < self.rows {
            Some((col, row))
        } else {
            None
        }
    }
}

#[derive(Debug, Default, Copy, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NavFreeGridConnection {
    pub from: (isize, isize),
    pub to: (isize, isize),
}

/// Nav free grid identifier.
pub type NavFreeGridID = ID<NavFreeGrid>;

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct NavFreeGrid {
    id: NavFreeGridID,
    cells: Vec<(isize, isize)>,
    costs: Vec<Scalar>,
    graph: Graph<(), (), Undirected>,
    nodes: Vec<NodeIndex>,
    nodes_map: HashMap<NodeIndex, usize>,
}

impl NavFreeGrid {
    pub fn new(connections: Vec<NavFreeGridConnection>) -> Self {
        let cells = connections
            .iter()
            .map(|c| c.from)
            .chain(connections.iter().map(|c| c.to))
            .collect::<HashSet<_>>()
            .into_iter()
            .collect::<Vec<_>>();
        let costs = vec![1.0; cells.len()];
        let mut graph = Graph::<(), (), Undirected>::with_capacity(cells.len(), connections.len());
        let nodes = (0..cells.len())
            .map(|_| graph.add_node(()))
            .collect::<Vec<_>>();
        for connection in connections {
            let ia = cells.iter().position(|c| connection.from == *c);
            let ib = cells.iter().position(|c| connection.to == *c);
            if let (Some(ia), Some(ib)) = (ia, ib) {
                graph.add_edge(nodes[ia], nodes[ib], ());
            }
        }
        let nodes_map = iter!(nodes).enumerate().map(|(i, n)| (*n, i)).collect();
        Self {
            id: NavFreeGridID::new(),
            cells,
            costs,
            graph,
            nodes,
            nodes_map,
        }
    }

    #[inline]
    pub fn id(&self) -> NavFreeGridID {
        self.id
    }

    #[inline]
    pub fn cells(&self) -> &[(isize, isize)] {
        &self.cells
    }

    #[inline]
    pub fn cells_costs(&self) -> &[Scalar] {
        &self.costs
    }

    #[inline]
    pub fn set_cell_cost(&mut self, col: isize, row: isize, cost: Scalar) -> Option<Scalar> {
        let index = self.index(col, row)?;
        let c = self.costs.get_mut(index)?;
        let old = *c;
        *c = cost.max(0.0);
        Some(old)
    }

    pub fn neighbors(
        &self,
        col: isize,
        row: isize,
    ) -> Option<impl Iterator<Item = (isize, isize)> + '_> {
        let index = self.index(col, row)?;
        let node = self.nodes[index];
        Some(self.graph.neighbors(node).filter_map(|node| {
            self.nodes_map
                .get(&node)
                .and_then(|index| self.coord(*index))
        }))
    }

    pub fn find_path(
        &self,
        from: (isize, isize),
        to: (isize, isize),
    ) -> Option<Vec<(isize, isize)>> {
        self.find_path_custom(from, to, |_, _| true)
    }

    // filter params: first col-row, second col-row.
    pub fn find_path_custom<F>(
        &self,
        from: (isize, isize),
        to: (isize, isize),
        mut filter: F,
    ) -> Option<Vec<(isize, isize)>>
    where
        F: FnMut((isize, isize), (isize, isize)) -> bool,
    {
        let start_index = self.index(from.0, from.1)?;
        let end_index = self.index(to.0, to.1)?;
        let start_node = *self.nodes.get(start_index)?;
        let end_node = *self.nodes.get(end_index)?;
        let nodes = astar(
            &self.graph,
            start_node,
            |n| n == end_node,
            |e| {
                let a = self.nodes_map[&e.source()];
                let b = self.nodes_map[&e.target()];
                if filter(self.coord(a).unwrap(), self.coord(b).unwrap()) {
                    let a = self.costs[a];
                    let b = self.costs[b];
                    a * b
                } else {
                    SCALAR_MAX
                }
            },
            |_| 0.0,
        )?
        .1;
        Some(
            nodes
                .into_iter()
                .filter_map(|n| self.coord(self.nodes_map[&n]))
                .collect::<Vec<_>>(),
        )
    }

    pub fn find_islands(&self) -> Vec<Vec<(isize, isize)>> {
        tarjan_scc(&self.graph)
            .into_iter()
            .map(|v| {
                v.into_iter()
                    .filter_map(|n| self.nodes_map.get(&n).and_then(|i| self.coord(*i)))
                    .collect::<Vec<_>>()
            })
            .filter(|v| !v.is_empty())
            .collect()
    }

    pub fn index(&self, col: isize, row: isize) -> Option<usize> {
        let coord = (col, row);
        self.cells.iter().position(|c| coord == *c)
    }

    pub fn coord(&self, index: usize) -> Option<(isize, isize)> {
        self.cells.get(index).copied()
    }
}
