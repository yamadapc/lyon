use std::cmp::Ordering;
use std::mem::swap;
use std::usize;
use std::f32;

use crate::math::*;
use crate::{EndpointId, CtrlPointId};
use crate::events::{PathEvent};
use crate::id_path;
use crate::Position;
use crate::geom::QuadraticBezierSegment;

fn compare_positions(a: Point, b: Point) -> Ordering {
    if a.y > b.y {
        return Ordering::Greater;
    }
    if a.y < b.y {
        return Ordering::Less;
    }
    if a.x > b.x {
        return Ordering::Greater;
    }
    if a.x < b.x {
        return Ordering::Less;
    }
    return Ordering::Equal;
}

#[inline]
fn is_after(a: Point, b: Point) -> bool {
    a.y > b.y || (a.y == b.y && a.x > b.x)
}

pub struct TraversalEvent {
    next_sibling: usize,
    next_event: usize,
    position: Point,
}

#[derive(Clone, Debug)]
struct EdgeData {
    from: EndpointId,
    to: EndpointId,
    ctrl: CtrlPointId,
    range: std::ops::Range<f32>,
    winding: i16,
}

pub struct Traversal {
    events: Vec<TraversalEvent>,
    edge_data: Vec<EdgeData>,
    first: usize,
    sorted: bool,
}

impl Traversal {
    pub fn new() -> Self {
        Traversal {
            events: Vec::new(),
            edge_data: Vec::new(),
            first: 0,
            sorted: false,
        }
    }

    pub fn with_capacity(cap: usize) -> Self {
        Traversal {
            events: Vec::with_capacity(cap),
            edge_data: Vec::with_capacity(cap),
            first: 0,
            sorted: false,
        }
    }

    pub fn from_id_path<Ep, Cp>(path: id_path::IdPathSlice<Ep, Cp>) -> Self
    where
        Ep: Position,
        Cp: Position,
    {
        let mut builder = TraversalBuilder::new();
        builder.set_id_path(path);
        builder.build()
    }

    pub fn reserve(&mut self, n: usize) {
        self.events.reserve(n);
    }

    pub fn push(&mut self, position: Point) {
        let next_event = self.events.len() + 1;
        self.events.push(TraversalEvent {
            position,
            next_sibling: usize::MAX,
            next_event,
        });
        self.sorted = false;
    }

    pub fn clear(&mut self) {
        self.events.clear();
        self.first = 0;
        self.sorted = false;
    }

    pub fn first_id(&self) -> usize { self.first }

    pub fn next_id(&self, id: usize) -> usize { self.events[id].next_event }

    pub fn next_sibling_id(&self, id: usize) -> usize { self.events[id].next_sibling }

    pub fn valid_id(&self, id: usize) -> bool { id < self.events.len() }

    pub fn position(&self, id: usize) -> Point { self.events[id].position }

    pub fn sort(&mut self) {
        // This is more or less a bubble-sort, the main difference being that elements with the same
        // position are grouped in a "sibling" linked list.

        if self.sorted {
            return;
        }
        self.sorted = true;

        if self.events.len() <= 1 {
            return;
        }

        let mut current = 0;
        let mut prev = 0;
        let mut last = self.events.len() - 1;
        let mut swapped = false;

        #[cfg(test)]
        let mut iter_count = self.events.len() * self.events.len();

        loop {
            #[cfg(test)] {
                assert!(iter_count > 0);
                iter_count -= 1;
            }

            let rewind = current == last ||
                !self.valid_id(current) ||
                !self.valid_id(self.next_id(current));

            if rewind {
                last = prev;
                prev = self.first;
                current = self.first;
                if !swapped || last == self.first {
                    return;
                }
                swapped = false;
            }

            let next = self.next_id(current);
            let a = self.events[current].position;
            let b = self.events[next].position;
            match compare_positions(a, b) {
                Ordering::Less => {
                    // Already ordered.
                    prev = current;
                    current = next;
                }
                Ordering::Greater => {
                    // Need to swap current and next.
                    if prev != current && prev != next {
                        self.events[prev].next_event = next;
                    }
                    if current == self.first {
                        self.first = next;
                    }
                    if next == last {
                        last = current;
                    }
                    let next_next = self.next_id(next);
                    self.events[current].next_event = next_next;
                    self.events[next].next_event = current;
                    swapped = true;
                    prev = next;
                }
                Ordering::Equal => {
                    // Append next to current's sibling list.
                    let next_next = self.next_id(next);
                    self.events[current].next_event = next_next;
                    let mut current_sibling = current;
                    let mut next_sibling = self.next_sibling_id(current);
                    while self.valid_id(next_sibling) {
                        current_sibling = next_sibling;
                        next_sibling = self.next_sibling_id(current_sibling);
                    }
                    self.events[current_sibling].next_sibling = next;
                }
            }
        }
    }

    #[allow(dead_code)]
    fn log(&self) {
        let mut iter_count = self.events.len() * self.events.len();

        println!("--");
        let mut current = self.first;
        while current < self.events.len() {
            assert!(iter_count > 0);
            iter_count -= 1;

            print!("[");
            let mut current_sibling = current;
            while current_sibling < self.events.len() {
                print!("{:?},", self.events[current_sibling].position);
                current_sibling = self.events[current_sibling].next_sibling;
            }
            print!("]  ");
            current = self.events[current].next_event;
        }
        println!("\n--");
    }

    #[allow(dead_code)]
    fn assert_sorted(&self) {
        let mut current = self.first;
        let mut pos = point(f32::MIN, f32::MIN);
        while self.valid_id(current) {
            assert!(is_after(self.events[current].position, pos));
            pos = self.events[current].position;
            let mut current_sibling = current;
            while self.valid_id(current_sibling) {
                assert_eq!(self.events[current_sibling].position, pos);
                current_sibling = self.next_sibling_id(current_sibling);
            }
            current = self.next_id(current);
        }
    }
}

struct TraversalBuilder {
    current: Point,
    current_id: EndpointId,
    first: Point,
    prev: Point,
    second: Point,
    nth: u32,
    tx: Traversal,
}

impl TraversalBuilder {
    fn new() -> Self {
        Self::with_capacity(0)
    }

    fn with_capacity(cap: usize) -> Self {
        TraversalBuilder {
            current: point(f32::NAN, f32::NAN),
            first: point(f32::NAN, f32::NAN),
            prev: point(f32::NAN, f32::NAN),
            second: point(f32::NAN, f32::NAN),
            current_id: EndpointId::INVALID,
            nth: 0,
            tx: Traversal::with_capacity(cap),
        }
    }


    fn set_id_path<Endpoint, CtrlPoint>(&mut self, path: id_path::IdPathSlice<Endpoint, CtrlPoint>)
    where
        Endpoint: Position,
        CtrlPoint: Position,
    {
        for event in path.id_iter() {
            match event {
                PathEvent::Line { to, .. } => {
                    let to_pos = path[to].position();
                    self.monotonic_edge(to_pos, CtrlPointId::INVALID, to, 0.0..1.0);
                }
                PathEvent::Quadratic { ctrl, to, .. } => {
                    let segment = QuadraticBezierSegment {
                        from: self.current,
                        ctrl: path[ctrl].position(),
                        to: path[to].position(),
                    };
                    segment.for_each_monotonic_range(|range| {
                        let position = segment.sample(range.end);
                        self.monotonic_edge(position, ctrl, to, range);
                    });
                }
                PathEvent::Begin { at } => {
                    let pos = path[at].position();
                    self.begin(pos, at);
                }
                PathEvent::End { last, first, .. } => {
                    let first_pos = path[first].position();
                    self.end(last, first_pos, first);
                }
                _ => unimplemented!(),
            }
        }
    }

    fn monotonic_edge(
        &mut self,
        to: Point,
        ctrl_id: CtrlPointId,
        mut to_id: EndpointId,
        range: std::ops::Range<f32>,
    ) {
        if self.current == to {
            return;
        }

        let next_id = to_id;
        let mut from = self.current;
        let mut from_id = self.current_id;
        let mut winding = 1;
        if is_after(from, to) {
            if self.nth > 0 && is_after(from, self.prev) {
                self.vertex_event(from);
            }

            from = to;
            swap(&mut from_id, &mut to_id);
            winding = -1;
        }

        //println!("Edge {:?}/{:?} {:?} ->", from_id, to_id, from);
        debug_assert!(from_id != EndpointId::INVALID);
        debug_assert!(to_id != EndpointId::INVALID);
        self.tx.push(from);
        self.tx.edge_data.push(EdgeData {
            from: from_id,
            ctrl: ctrl_id,
            to: to_id,
            range,
            winding,
        });

        if self.nth == 0 {
            self.second = to;
        }

        self.nth += 1;
        self.prev = self.current;
        self.current = to;
        self.current_id = next_id;
    }

    fn vertex_event(&mut self, at: Point) {
        self.tx.push(at);
        self.tx.edge_data.push(EdgeData {
            from: EndpointId::INVALID,
            ctrl: CtrlPointId::INVALID,
            to: EndpointId::INVALID,
            range: 0.0..1.0,
            winding: 0,
        });
    }

    fn end(&mut self, last_id: EndpointId, first: Point, first_id: EndpointId) {
        if self.nth == 0 {
            return;
        }

        // Unless we are already back to the first point we no need to
        // to insert an edge.
        // TODO: should check the position instead of ID?
        if last_id != first_id {
            let first_id = first_id;
            self.monotonic_edge(first, CtrlPointId::INVALID, first_id, 0.0..1.0)
        }

        // Since we can only check for the need of a vertex event when
        // we have a previous edge, we skipped it for the first edge
        // and have to do it now.
        if is_after(first, self.prev) && is_after(first, self.second) {
            self.vertex_event(first);
        }

        self.nth = 0;
    }

    fn begin(&mut self, to: Point, to_id: EndpointId) {
        self.nth = 0;
        self.first = to;
        self.current = to;
        self.current_id = to_id;
    }

    fn build(mut self) -> Traversal {
        self.tx.sort();
        self.tx
    }
}

#[test]
fn test_traversal_sort_1() {
    let mut tx = Traversal::new();
    tx.push(point(0.0, 0.0));
    tx.push(point(4.0, 0.0));
    tx.push(point(2.0, 0.0));
    tx.push(point(3.0, 0.0));
    tx.push(point(4.0, 0.0));
    tx.push(point(0.0, 0.0));
    tx.push(point(6.0, 0.0));

    tx.sort();
    tx.assert_sorted();
}

#[test]
fn test_traversal_sort_2() {
    let mut tx = Traversal::new();
    tx.push(point(0.0, 0.0));
    tx.push(point(0.0, 0.0));
    tx.push(point(0.0, 0.0));
    tx.push(point(0.0, 0.0));

    tx.sort();
    tx.assert_sorted();
}

#[test]
fn test_traversal_sort_3() {
    let mut tx = Traversal::new();
    tx.push(point(0.0, 0.0));
    tx.push(point(1.0, 0.0));
    tx.push(point(2.0, 0.0));
    tx.push(point(3.0, 0.0));
    tx.push(point(4.0, 0.0));
    tx.push(point(5.0, 0.0));

    tx.sort();
    tx.assert_sorted();
}

#[test]
fn test_traversal_sort_4() {
    let mut tx = Traversal::new();
    tx.push(point(5.0, 0.0));
    tx.push(point(4.0, 0.0));
    tx.push(point(3.0, 0.0));
    tx.push(point(2.0, 0.0));
    tx.push(point(1.0, 0.0));
    tx.push(point(0.0, 0.0));

    tx.sort();
    tx.assert_sorted();
}

#[test]
fn test_traversal_sort_5() {
    let mut tx = Traversal::new();
    tx.push(point(5.0, 0.0));
    tx.push(point(5.0, 0.0));
    tx.push(point(4.0, 0.0));
    tx.push(point(4.0, 0.0));
    tx.push(point(3.0, 0.0));
    tx.push(point(3.0, 0.0));
    tx.push(point(2.0, 0.0));
    tx.push(point(2.0, 0.0));
    tx.push(point(1.0, 0.0));
    tx.push(point(1.0, 0.0));
    tx.push(point(0.0, 0.0));
    tx.push(point(0.0, 0.0));

    tx.sort();
    tx.assert_sorted();
}
