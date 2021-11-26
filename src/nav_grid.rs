use crate::{Error, NavResult, Scalar};
use petgraph::{algo::astar, graph::NodeIndex, visit::EdgeRef, Graph, Undirected};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
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

/// Nav grid identifier.
pub type NavGridID = ID<NavGrid>;

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct NavGrid {
    id: NavGridID,
    cols: usize,
    rows: usize,
    cells: Vec<bool>,
    costs: Vec<Scalar>,
    graph: Graph<(), (), Undirected>,
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

        let mut graph = Graph::<(), (), Undirected>::new_undirected();
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
        graph.extend_with_edges((0..((cols - 1) * rows)).filter_map(|i| {
            let ca = i % (cols - 1);
            let cb = ca + 1;
            let r = i / (cols - 1);
            let ia = r * cols + ca;
            let ib = r * cols + cb;
            let na = nodes[ia];
            let nb = nodes[ib];
            if let (Some(na), Some(nb)) = (na, nb) {
                Some((na, nb, ()))
            } else {
                None
            }
        }));
        graph.extend_with_edges((0..(cols * (rows - 1))).filter_map(|i| {
            let c = i % cols;
            let ra = i / cols;
            let rb = ra + 1;
            let ia = ra * cols + c;
            let ib = rb * cols + c;
            let na = nodes[ia]?;
            let nb = nodes[ib]?;
            Some((na, nb, ()))
        }));
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
