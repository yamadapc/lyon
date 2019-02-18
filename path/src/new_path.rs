//! The default path data structure.

use builder::*;

use PathEvent;
use math::*;
use geom::{LineSegment, QuadraticBezierSegment, CubicBezierSegment, Arc};

use std::iter::IntoIterator;
use std::ops;
use std::mem;

pub enum Event<'l, Endpoint, CtrlPoint> {
    StartSubPath,
    Line { from: &'l Endpoint, to: &'l Endpoint },
    Quadratic { from: &'l Endpoint, ctrl: &'l CtrlPoint, to: &'l Endpoint },
    Cubic { from: &'l Endpoint, ctrl1: &'l CtrlPoint, ctrl2: &'l CtrlPoint, to: &'l Endpoint },
    Close { from: &'l Endpoint, to: &'l Endpoint },
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct EdgeId(u32);
impl EdgeId {
    pub fn to_usize(&self) -> usize { self.0 as usize }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct CtrlPointId(u32);
impl CtrlPointId {
    pub fn to_usize(&self) -> usize { self.0 as usize }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct EndpointId(u32);
impl EndpointId {
    pub fn to_usize(&self) -> usize { self.0 as usize }
}

/// Enumeration corresponding to the [PathEvent](https://docs.rs/lyon_core/*/lyon_core/events/enum.PathEvent.html) enum
/// without the parameters.
///
/// This is used by the [Path](struct.Path.html) data structure to store path events a tad
/// more efficiently.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
enum Verb {
    MoveTo,
    LineTo,
    QuadraticTo,
    CubicTo,
    Close,
}

pub trait Vertex : Clone {
    fn position(&self) -> Point;
    fn set_position(&mut self, pos: Point);
    fn interpolate(a: &Self, b: &Self, t: f32) -> Self;
}

impl Vertex for Point {
    fn position(&self) -> Point { *self }
    fn set_position(&mut self, pos: Point) { *self = pos; }
    fn interpolate(a: &Self, b: &Self, t: f32) -> Self { a.lerp(*b, t) }
}

/// A simple path data structure.
///
/// It can be created using a [Builder](struct.Builder.html), and can be iterated over.
#[derive(Clone, Debug, Default)]
pub struct Path<Endpoint, CtrlPoint> {
    endpoints: Box<[Endpoint]>,
    ctrl_points: Box<[CtrlPoint]>,
    verbs: Box<[Verb]>,
}

/// A view on a `Path`.
#[derive(Copy, Clone, Debug)]
pub struct PathSlice<'l, Endpoint, CtrlPoint> {
    endpoints: &'l [Endpoint],
    ctrl_points: &'l [CtrlPoint],
    verbs: &'l [Verb],
}

impl<Endpoint: Vertex, CtrlPoint: Vertex> Path<Endpoint, CtrlPoint> {
    /// Creates a [Builder](struct.Builder.html) to create a path.
    pub fn builder() -> Builder<Endpoint, CtrlPoint> { Builder::new() }

    /// Creates an Empty `Path`.
    pub fn new() -> Self {
        Path {
            endpoints: Box::new([]),
            ctrl_points: Box::new([]),
            verbs: Box::new([]),
        }
    }

    /// Returns a view on this `Path`.
    pub fn as_slice(&self) -> PathSlice<Endpoint, CtrlPoint> {
        PathSlice {
            endpoints: &self.endpoints[..],
            ctrl_points: &self.ctrl_points[..],
            verbs: &self.verbs[..],
        }
    }

    /// Iterates over the entire `Path`.
    pub fn iter(&self) -> Iter<Endpoint, CtrlPoint> { Iter::new(&self.endpoints[..], &self.ctrl_points[..], &self.verbs[..]) }

    pub fn endpoints(&self) -> &[Endpoint] { &self.endpoints[..] }

    pub fn mut_endpoints(&mut self) -> &mut [Endpoint] { &mut self.endpoints[..] }

    pub fn ctrl_points(&self) -> &[CtrlPoint] { &self.ctrl_points[..] }

    pub fn mut_ctrl_points(&mut self) -> &mut [CtrlPoint] { &mut self.ctrl_points[..] }

    /// Concatenate two paths.
    pub fn merge(&self, other: &Self) -> Self {
        let mut verbs = Vec::with_capacity(self.verbs.len() + other.verbs.len());
        let mut endpoints = Vec::with_capacity(self.endpoints.len() + other.endpoints.len());
        let mut ctrl_points = Vec::with_capacity(self.ctrl_points.len() + other.ctrl_points.len());
        verbs.extend_from_slice(&self.verbs);
        verbs.extend_from_slice(&other.verbs);
        endpoints.extend_from_slice(&self.endpoints);
        endpoints.extend_from_slice(&other.endpoints);
        ctrl_points.extend_from_slice(&self.ctrl_points);
        ctrl_points.extend_from_slice(&other.ctrl_points);

        Path {
            verbs: verbs.into_boxed_slice(),
            endpoints: endpoints.into_boxed_slice(),
            ctrl_points: ctrl_points.into_boxed_slice(),
        }
    }
}

impl<'l, Endpoint: Vertex, CtrlPoint: Vertex> IntoIterator for &'l Path<Endpoint, CtrlPoint> {
    type Item = PathEvent;
    type IntoIter = Iter<'l, Endpoint, CtrlPoint>;

    fn into_iter(self) -> Iter<'l, Endpoint, CtrlPoint> { self.iter() }
}

impl<'l, Endpoint: Vertex, CtrlPoint: Vertex> Into<PathSlice<'l, Endpoint, CtrlPoint>> for &'l Path<Endpoint, CtrlPoint> {
    fn into(self) -> PathSlice<'l, Endpoint, CtrlPoint> {
        self.as_slice()
    }
}

/// An immutable view over a Path.
impl<'l, Endpoint, CtrlPoint> PathSlice<'l, Endpoint, CtrlPoint> {

    pub fn iter<'a>(&'a self) -> Iter<'l, Endpoint, CtrlPoint> {
        Iter::new(self.endpoints, self.ctrl_points, self.verbs)
    }

    pub fn endpoints(&self) -> &[Endpoint] { self.endpoints }
    pub fn ctrl_points(&self) -> &[CtrlPoint] { self.ctrl_points }
}

/// A view on a `Path` endoints and control points.
#[derive(Copy, Clone, Debug)]
pub struct PathDataSlice<'l, Endpoint, CtrlPoint> {
    endpoints: &'l [Endpoint],
    ctrl_points: &'l [CtrlPoint],
}


/// An iterator for `Path` and `PathSlice`.
#[derive(Clone, Debug)]
pub struct Iter<'l, Endpoint, CtrlPoint> {
    endpoints: ::std::slice::Iter<'l, Endpoint>,
    ctrl_points: ::std::slice::Iter<'l, CtrlPoint>,
    verbs: ::std::slice::Iter<'l, Verb>,
    current: Point,
    first: Point,
}

impl<'l, Endpoint, CtrlPoint> Iter<'l, Endpoint, CtrlPoint> {
    fn new(endpoints: &'l [Endpoint], ctrl_points: &'l [CtrlPoint], verbs: &'l [Verb]) -> Self {
        Iter {
            endpoints: endpoints.iter(),
            ctrl_points: ctrl_points.iter(),
            verbs: verbs.iter(),
            current: point(0.0, 0.0),
            first: point(0.0, 0.0),
        }
    }
}

impl<'l, Endpoint: Vertex, CtrlPoint: Vertex> Iterator for Iter<'l, Endpoint, CtrlPoint> {
    type Item = PathEvent;
    fn next(&mut self) -> Option<PathEvent> {
        match self.verbs.next() {
            Some(&Verb::MoveTo) => {
                self.current = self.endpoints.next().unwrap().position();
                self.first = self.current;
                Some(PathEvent::MoveTo(self.current))
            }
            Some(&Verb::LineTo) => {
                let from = self.current;
                self.current = self.endpoints.next().unwrap().position();
                Some(PathEvent::Line(LineSegment {
                    from, to: self.current
                }))
            }
            Some(&Verb::QuadraticTo) => {
                let from = self.current;
                let ctrl = self.ctrl_points.next().unwrap().position();
                self.current = self.endpoints.next().unwrap().position();
                Some(PathEvent::Quadratic(QuadraticBezierSegment {
                    from, ctrl, to: self.current
                }))
            }
            Some(&Verb::CubicTo) => {
                let from = self.current;
                let ctrl1 = self.ctrl_points.next().unwrap().position();
                let ctrl2 = self.ctrl_points.next().unwrap().position();
                self.current = self.endpoints.next().unwrap().position();
                Some(PathEvent::Cubic(CubicBezierSegment {
                    from, ctrl1, ctrl2, to: self.current
                }))
            }
            Some(&Verb::Close) => {
                let from = self.current;
                self.current = self.first;
                Some(PathEvent::Close(LineSegment {
                    from,
                    to: self.first,
                }))
            }
            None => None,
        }
    }
}

impl<'l, Endpoint: Vertex, CtrlPoint: Vertex> IntoIterator for PathSlice<'l, Endpoint, CtrlPoint> {
    type Item = PathEvent;
    type IntoIter = Iter<'l, Endpoint, CtrlPoint>;

    fn into_iter(self) -> Iter<'l, Endpoint, CtrlPoint> { self.iter() }
}

impl<'l, 'a, Endpoint: Vertex, CtrlPoint: Vertex> IntoIterator for &'a PathSlice<'l, Endpoint, CtrlPoint> {
    type Item = PathEvent;
    type IntoIter = Iter<'l, Endpoint, CtrlPoint>;

    fn into_iter(self) -> Iter<'l, Endpoint, CtrlPoint> { self.iter() }
}

pub struct Builder<Endpoint, CtrlPoint> {
    endpoints: Vec<Endpoint>,
    ctrl_points: Vec<CtrlPoint>,
    verbs: Vec<Verb>,
    first_endpoint: EndpointId,
    first_verb: u32,
    need_moveto: bool,
}

impl<Endpoint: Vertex, CtrlPoint: Vertex> Builder<Endpoint, CtrlPoint> {
    pub fn new() -> Self { Builder::with_capacity(128) }

    pub fn with_capacity(cap: usize) -> Self {
        Builder {
            endpoints: Vec::with_capacity(cap),
            ctrl_points: Vec::with_capacity(cap),
            verbs: Vec::with_capacity(cap),
            first_endpoint: EndpointId(0),
            first_verb: 0,
            need_moveto: true,
        }
    }

    pub fn move_to(&mut self, to: Endpoint) {
        self.need_moveto = false;
        self.first_endpoint = EndpointId(self.endpoints.len() as u32);
        self.first_verb = self.verbs.len() as u32;
        self.endpoints.push(to);
        self.verbs.push(Verb::MoveTo);
    }

    pub fn line_to(&mut self, to: Endpoint) {
        self.move_to_if_needed();
        self.endpoints.push(to);
        self.verbs.push(Verb::LineTo);
    }

    pub fn close(&mut self) {
        // Relative path ops tend to accumulate small floating point imprecisions
        // which results in the last segment ending almost but not quite at the
        // start of the sub-path, causing a new edge to be inserted which often
        // intersects with the first or last edge. This can affect algorithms that
        // Don't handle self-intersecting paths.
        // Deal with this by snapping the last point if it is very close to the
        // start of the sub path.
        let first_position = self.endpoints[self.first_endpoint.to_usize()].position();
        if let Some(p) = self.endpoints.last_mut() {
            let d = (p.position() - first_position).abs();
            if d.x + d.y < 0.0001 {
                p.set_position(first_position);
            }
        }

        self.verbs.push(Verb::Close);
        self.need_moveto = true;
    }

    pub fn quadratic_bezier_to(&mut self, ctrl: CtrlPoint, to: Endpoint) {
        self.move_to_if_needed();
        self.ctrl_points.push(ctrl);
        self.endpoints.push(to);
        self.verbs.push(Verb::QuadraticTo);
    }

    pub fn cubic_bezier_to(&mut self, ctrl1: CtrlPoint, ctrl2: CtrlPoint, to: Endpoint) {
        self.move_to_if_needed();
        self.ctrl_points.push(ctrl1);
        self.ctrl_points.push(ctrl2);
        self.endpoints.push(to);
        self.verbs.push(Verb::CubicTo);
    }

    pub fn current_position(&self) -> Point {
        self.endpoints.last()
            .map(|p| p.position())
            .unwrap_or_else(|| point(0.0, 0.0))
    }

    pub fn build(self) -> Path<Endpoint, CtrlPoint> {
        Path {
            endpoints: self.endpoints.into_boxed_slice(),
            ctrl_points: self.ctrl_points.into_boxed_slice(),
            verbs: self.verbs.into_boxed_slice(),
        }
    }

    fn move_to_if_needed(&mut self) {
        if self.need_moveto {
            let first = self.endpoints[self.first_endpoint.to_usize()].clone();
            self.move_to(first);
        }
    }
}

#[test]
fn basic() {
    let mut builder = Path::builder();
    builder.move_to(point(0.0, 0.0));
    builder.line_to(point(10.0, 0.0));
    builder.quadratic_bezier_to(point(5.0, 5.0), point(0.0, 0.0));
    builder.close();
    let path = builder.build();
}

#[test]
fn custom_vertices() {
    #[derive(Clone, Debug)]
    pub struct MyEndpoint {
        pos: Point,
        width: f32,
    };

    #[derive(Clone, Debug)]
    pub struct MyCtrlPoint {
        pos: Point,
    };

    impl Vertex for MyEndpoint {
        fn position(&self) -> Point { self.pos }
        fn set_position(&mut self, pos: Point) { self.pos = pos; }
        fn interpolate(a: &Self, b: &Self, t: f32) -> Self {
            MyEndpoint { pos: a.pos.lerp(b.pos, t), width: a.width * (1.0 - t) + b.width * t }
        }
    }
    impl Vertex for MyCtrlPoint {
        fn position(&self) -> Point { self.pos }
        fn set_position(&mut self, pos: Point) { self.pos = pos; }
        fn interpolate(a: &Self, b: &Self, t: f32) -> Self {
            MyCtrlPoint { pos: a.pos.lerp(b.pos, t) }
        }
    }

    let mut builder = Path::builder();
    builder.move_to(MyEndpoint { pos: point(0.0, 0.0), width: 1.0 });
    builder.line_to(MyEndpoint { pos: point(10.0, 0.0), width: 2.0 });
    builder.quadratic_bezier_to(
        MyCtrlPoint { pos: point(5.0, 5.0) },
        MyEndpoint { pos: point(0.0, 0.0), width: 1.0 },
    );
    builder.close();
    let path = builder.build();
}


#[derive(Clone, Debug)]
pub struct AdvancedPath<Endpoint, CtrlPoint> {
    endpoints: Vec<Endpoint>,
    ctrl_points: Vec<CtrlPoint>,
    edges: Vec<EdgeInfo>,
}

#[derive(Clone, Debug)]
struct EdgeInfo {
    from: EndpointId,
    to: EndpointId,
    ctrl1: CtrlPointId,
    ctrl2: CtrlPointId,
    next: EdgeId,
    prev: EdgeId,
}

/*

fn tess::tessellate(options, path: Into<Fill>, builder) -> Result

trait Builder<Ep, Cp> {
    fn add_vertex(&mut self, path: &PathDataSlice<Ep, Cp>, endpoint: EndpointId) -> Result<VertexId>;

    fn add_split_vertex(
        &mut self,
        path: &PathDataSlice<Ep, Cp>,
        edge: EdgeIds, t: f32,
    ) -> Result<VertexId>;

    fn add_intersection_vertex(
        &mut self,
        path: &PathDataSlice<Ep, Cp>,
        edge1: EdgeIds, t1: f32,
        edge2: EdgeIds, t2: f32,
    ) -> Result<VertexId>;

    fn add_triangle(&mut self, a: VertexId, b: VertexId, c: VertexId);

    fn add_curve_triangle(&mut self, a: VertexId, b: VertexId, c: VertexId);
}

*/