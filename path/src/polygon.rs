use crate::{EndpointId, CtrlPointId, PathEvent, Position, PositionStore};
use crate::math::Point;

/// A view over a sequence of endpoint IDs forming a polygon.
pub struct IdPolygonSlice<'l> {
    pub points: &'l[EndpointId],
    pub closed: bool,
}

impl<'l> IdPolygonSlice<'l> {
    // Returns an iterator over the endpoint IDs of the polygon.
    pub fn iter(&self) -> IdPolygonIter<'l> {
        IdPolygonIter {
            points: self.points.iter(),
            prev: None,
            first: EndpointId(0),
            closed: self.closed,
        }
    }
}

// An iterator of `PathEvent<EndpointId, ()>`.
pub struct IdPolygonIter<'l> {
    points: std::slice::Iter<'l, EndpointId>,
    prev: Option<EndpointId>,
    first: EndpointId,
    closed: bool,
}

impl<'l> Iterator for IdPolygonIter<'l> {
    type Item = PathEvent<EndpointId, ()>;
    fn next(&mut self) -> Option<PathEvent<EndpointId, ()>> {
        match (self.prev, self.points.next()) {
            (Some(from), Some(to)) => {
                self.prev = Some(*to);
                Some(PathEvent::Line { from, to: *to })
            }
            (None, Some(at)) => {
                self.prev = Some(*at);
                self.first = *at;
                Some(PathEvent::Begin { at: *at })
            }
            (Some(last), None) => {
                self.prev = None;
                Some(PathEvent::End {
                    last,
                    first: self.first,
                    close: self.closed,
                })
            }
            (None, None) => None,
        }
    }
}

/// A view over a sequence of endpoints forming a polygon.
pub struct PolygonSlice<'l, T> {
    pub points: &'l[T],
    pub closed: bool,
}

impl<'l, T> PolygonSlice<'l, T> {
    pub fn iter(&self) -> PolygonIter<'l, T> {
        PolygonIter {
            points: self.points.iter(),
            prev: None,
            first: None,
            closed: self.closed,
        }
    }
}

// An iterator of `PathEvent<&Endpoint, ()>`.
pub struct PolygonIter<'l, T> {
    points: std::slice::Iter<'l, T>,
    prev: Option<&'l T>,
    first: Option<&'l T>,
    closed: bool,
}

impl<'l, T> Iterator for PolygonIter<'l, T> {
    type Item = PathEvent<&'l T, ()>;
    fn next(&mut self) -> Option<PathEvent<&'l T, ()>> {
        match (self.prev, self.points.next()) {
            (Some(from), Some(to)) => {
                self.prev = Some(to);
                Some(PathEvent::Line { from, to })
            }
            (None, Some(at)) => {
                self.prev = Some(at);
                self.first = Some(at);
                Some(PathEvent::Begin { at })
            }
            (Some(last), None) => {
                self.prev = None;
                Some(PathEvent::End {
                    last,
                    first: self.first.unwrap(),
                    close: self.closed,
                })
            }
            (None, None) => None,
        }
    }
}

impl<'l, Endpoint> PositionStore for PolygonSlice<'l, Endpoint>
where
    Endpoint: Position,
{
    fn endpoint_position(&self, id: EndpointId) -> Point {
        self.points[id.to_usize()].position()
    }

    fn ctrl_point_position(&self, _: CtrlPointId) -> Point {
        panic!("Polygons do not have control points.");
    }
}
