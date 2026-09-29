"""
OSM Loader module for campus road network, junctions, and named stops.
"""

import math
import os
import xml.etree.ElementTree as ET
import yaml
from collections import defaultdict, deque


class CampusMap:
    def __init__(self, osm_path, yaml_path=None):
        self.osm_path = osm_path
        self.yaml_path = yaml_path
        
        # Campus reference datum for local Cartesian projection (SW corner of campus)
        self.lat0 = 30.960
        self.lon0 = 76.465
        self.earth_r = 6371000.0

        self.nodes = {}          # node_id -> (lat, lon)
        self.xy_nodes = {}       # node_id -> (x, y)
        self.road_graph = defaultdict(dict)  # u -> {v: distance_meters}
        self.road_ways = []      # list of node_id lists
        self.main_nodes = set()  # node_ids in main connected component
        self.junctions = {}      # node_id -> {'type': str, 'degree': int, 'lat': float, 'lon': float, 'x': float, 'y': float}
        self.stops = {}          # stop_name -> dict of stop info
        self.building_polygons = []  # list of (name, height, [(x,y), ...])

        self._load_osm()
        self._load_or_build_stops()

    def to_xy(self, lat, lon):
        """Project GPS lat/lon to local Cartesian (x, y) in meters."""
        x = self.earth_r * math.radians(lon - self.lon0) * math.cos(math.radians(self.lat0))
        y = self.earth_r * math.radians(lat - self.lat0)
        return round(x, 2), round(y, 2)

    def to_latlon(self, x, y):
        """Convert local Cartesian (x, y) back to GPS lat/lon."""
        lat = self.lat0 + math.degrees(y / self.earth_r)
        lon = self.lon0 + math.degrees(x / (self.earth_r * math.cos(math.radians(self.lat0))))
        return round(lat, 7), round(lon, 7)

    def _load_osm(self):
        if not os.path.exists(self.osm_path):
            raise FileNotFoundError(f"OSM file not found: {self.osm_path}")

        tree = ET.parse(self.osm_path)
        root = tree.getroot()

        # Parse nodes
        for n in root.findall('node'):
            if n.get('action') == 'delete':
                continue
            nid = n.get('id')
            lat = float(n.get('lat'))
            lon = float(n.get('lon'))
            self.nodes[nid] = (lat, lon)
            self.xy_nodes[nid] = self.to_xy(lat, lon)

        # Parse road ways
        for w in root.findall('way'):
            if w.get('action') == 'delete':
                continue
            tags = {t.get('k'): t.get('v') for t in w.findall('tag')}
            if 'highway' in tags:
                refs = [nd.get('ref') for nd in w.findall('nd') if nd.get('ref') in self.nodes]
                if len(refs) > 1:
                    self.road_ways.append(refs)
                    for i in range(len(refs) - 1):
                        u, v = refs[i], refs[i + 1]
                        p1 = self.nodes[u]
                        p2 = self.nodes[v]
                        dlat = math.radians(p2[0] - p1[0]) * self.earth_r
                        dlon = math.radians(p2[1] - p1[1]) * self.earth_r * math.cos(math.radians((p1[0] + p2[0]) / 2))
                        dist = math.hypot(dlat, dlon)
                        self.road_graph[u][v] = dist
                        self.road_graph[v][u] = dist

        # Parse building footprints
        for w in root.findall('way'):
            if w.get('action') == 'delete':
                continue
            tags = {t.get('k'): t.get('v') for t in w.findall('tag')}
            if 'building' in tags:
                refs = [nd.get('ref') for nd in w.findall('nd') if nd.get('ref') in self.nodes]
                if len(refs) > 2:
                    pts = [self.xy_nodes[r] for r in refs]
                    try:
                        h = float(tags.get('height', '12.0'))
                    except ValueError:
                        h = 12.0
                    name = tags.get('name', '')
                    self.building_polygons.append((name, h, pts))

        # Find largest connected component
        visited = set()
        components = []
        for node in self.road_graph:
            if node not in visited:
                comp = []
                q = deque([node])
                visited.add(node)
                while q:
                    curr = q.popleft()
                    comp.append(curr)
                    for nbr in self.road_graph[curr]:
                        if nbr not in visited:
                            visited.add(nbr)
                            q.append(nbr)
                components.append(comp)

        components.sort(key=len, reverse=True)
        self.main_nodes = set(components[0]) if components else set()

        # Extract junctions in main road network (degree != 2)
        for n in self.main_nodes:
            deg = len(self.road_graph[n])
            if deg != 2:
                lat, lon = self.nodes[n]
                x, y = self.xy_nodes[n]
                jtype = '4-way Crossroad' if deg == 4 else ('3-way T/Y Junction' if deg == 3 else 'Road End / Turnaround')
                self.junctions[n] = {
                    'degree': deg,
                    'type': jtype,
                    'lat': round(lat, 7),
                    'lon': round(lon, 7),
                    'x': x,
                    'y': y
                }

    def _load_or_build_stops(self):
        # 1. Extract all named stops and amenities directly from the OSM XML
        if os.path.exists(self.osm_path):
            try:
                tree = ET.parse(self.osm_path)
                root = tree.getroot()
                for n in root.findall('node'):
                    if n.get('action') == 'delete':
                        continue
                    tags = {t.get('k'): t.get('v') for t in n.findall('tag')}
                    name = tags.get('name')
                    if name and (tags.get('highway') == 'bus_stop' or tags.get('amenity') == 'bus_station' or 'stop' in tags or 'amenity' in tags):
                        if not name.startswith('Junction'):
                            lat, lon = float(n.get('lat')), float(n.get('lon'))
                            sx, sy = self.to_xy(lat, lon)
                            best_nid = min(self.main_nodes, key=lambda nid: math.hypot(self.xy_nodes[nid][0] - sx, self.xy_nodes[nid][1] - sy)) if self.main_nodes else None
                            rx, ry = self.xy_nodes[best_nid] if best_nid else (sx, sy)
                            self.stops[name] = {
                                'lat': lat, 'lon': lon, 'x': sx, 'y': sy,
                                'snapped_road_node_id': best_nid,
                                'road_node_lat': self.nodes[best_nid][0] if best_nid else lat,
                                'road_node_lon': self.nodes[best_nid][1] if best_nid else lon,
                                'road_node_x': rx, 'road_node_y': ry,
                                'distance_to_road_m': round(math.hypot(rx - sx, ry - sy), 1)
                            }
            except Exception as e:
                pass

        # 2. Merge with YAML stops if present
        if self.yaml_path and os.path.exists(self.yaml_path):
            try:
                with open(self.yaml_path, 'r') as f:
                    data = yaml.safe_load(f)
                    if 'campus_stops' in data:
                        for k, v in data['campus_stops'].items():
                            if k not in self.stops:
                                self.stops[k] = v
            except Exception:
                pass

        # 3. Fallback defaults if still empty
        if not self.stops:
            default_stops = {
                'Satluj Front': (30.96825, 76.46840),
                'Satluj Back': (30.96925, 76.46840),
                'Beas Front': (30.96825, 76.46700),
                'Chenab Front': (30.96922, 76.46550),
                'Mess': (30.96618, 76.47130),
                'Ravi Front': (30.96780, 76.47000),
                'Ravi Back': (30.96741, 76.46965),
                'Chemistry Block': (30.96868, 76.47060),
                'Mechanical Block Front': (30.96750, 76.47111),
                'Mechanical Block Back': (30.96650, 76.47111),
                'Auditorium': (30.96860, 76.47359),
                'SAB A': (30.96786, 76.47376),
                'SAB B': (30.96781, 76.47489),
                'SAB S': (30.96680, 76.47450),
                'Admin': (30.96860, 76.47262),
                'LOC': (30.96772, 76.47195),
                'Main Gate': (30.96009, 76.47441),
                'Nalanda Library': (30.96763, 76.47320),
                'Maggi Point': (30.96813, 76.46939),
                'Burger House': (30.96811, 76.46939)
            }
            for name, (lat, lon) in default_stops.items():
                sx, sy = self.to_xy(lat, lon)
                best_nid = min(self.main_nodes, key=lambda nid: math.hypot(self.xy_nodes[nid][0] - sx, self.xy_nodes[nid][1] - sy))
                rx, ry = self.xy_nodes[best_nid]
                self.stops[name] = {
                    'lat': lat, 'lon': lon, 'x': sx, 'y': sy,
                    'snapped_road_node_id': best_nid,
                    'road_node_lat': self.nodes[best_nid][0],
                    'road_node_lon': self.nodes[best_nid][1],
                    'road_node_x': rx, 'road_node_y': ry,
                    'distance_to_road_m': round(math.hypot(rx - sx, ry - sy), 1)
                }

        # Aliases for common stop naming conventions
        if 'SAB S' in self.stops and 'SAB C' not in self.stops:
            self.stops['SAB C'] = dict(self.stops['SAB S'])

        # 4. Snap all stops to nearest road segment (edge-based projection)
        self._snap_all_stops_to_road_segments()

    def _snap_all_stops_to_road_segments(self):
        """
        Projects each stop onto the nearest road segment in road_ways.
        If a stop is closest to the middle of a segment (e.g. LOC on the upper road),
        inserts a synthetic road node on that segment and updates the road graph
        and road ways so routes start/end right at the stop.
        """
        def point_to_segment(p, a, b):
            px, py = p
            ax, ay = a
            bx, by = b
            dx, dy = bx - ax, by - ay
            if dx == 0 and dy == 0:
                return math.hypot(px - ax, py - ay), a, 0.0
            t = max(0.0, min(1.0, ((px - ax)*dx + (py - ay)*dy) / (dx*dx + dy*dy)))
            proj = (ax + t*dx, ay + t*dy)
            return math.hypot(px - proj[0], py - proj[1]), proj, t

        for name, info in list(self.stops.items()):
            p = (info['x'], info['y'])
            best_d = float('inf')
            best_edge = None
            best_proj = None
            best_t = 0.0

            for refs in self.road_ways:
                for i in range(len(refs) - 1):
                    u, v = refs[i], refs[i + 1]
                    if u in self.xy_nodes and v in self.xy_nodes:
                        d, proj, t = point_to_segment(p, self.xy_nodes[u], self.xy_nodes[v])
                        if d < best_d:
                            best_d = d
                            best_edge = (u, v)
                            best_proj = proj
                            best_t = t

            if best_edge is None:
                continue

            u, v = best_edge
            if best_t <= 0.02:
                chosen_nid = u
                rx, ry = self.xy_nodes[u]
            elif best_t >= 0.98:
                chosen_nid = v
                rx, ry = self.xy_nodes[v]
            else:
                # Insert synthetic road node directly on this road segment
                safe_name = name.replace(' ', '_').replace('&', '_').replace('.', '_')
                chosen_nid = f"stop_node_{safe_name}"
                rx, ry = best_proj
                self.xy_nodes[chosen_nid] = (rx, ry)
                self.nodes[chosen_nid] = self.to_latlon(rx, ry)
                self.main_nodes.add(chosen_nid)

                du = math.hypot(rx - self.xy_nodes[u][0], ry - self.xy_nodes[u][1])
                dv = math.hypot(rx - self.xy_nodes[v][0], ry - self.xy_nodes[v][1])

                if u in self.road_graph and v in self.road_graph[u]:
                    del self.road_graph[u][v]
                if v in self.road_graph and u in self.road_graph[v]:
                    del self.road_graph[v][u]

                if chosen_nid not in self.road_graph:
                    self.road_graph[chosen_nid] = {}
                self.road_graph[chosen_nid][u] = du
                self.road_graph[chosen_nid][v] = dv
                self.road_graph[u][chosen_nid] = du
                self.road_graph[v][chosen_nid] = dv

                for refs in self.road_ways:
                    for i in range(len(refs) - 1):
                        if refs[i] == u and refs[i + 1] == v:
                            refs.insert(i + 1, chosen_nid)
                            break
                        elif refs[i] == v and refs[i + 1] == u:
                            refs.insert(i + 1, chosen_nid)
                            break

            info['snapped_road_node_id'] = chosen_nid
            info['road_node_x'] = rx
            info['road_node_y'] = ry
            info['distance_to_road_m'] = round(best_d, 1)

    def snap_to_road(self, x, y):
        """Find the closest road network node ID to a given (x, y) point."""
        if not self.main_nodes:
            return None, float('inf')
        best_nid = min(self.main_nodes, key=lambda nid: math.hypot(self.xy_nodes[nid][0] - x, self.xy_nodes[nid][1] - y))
        dist = math.hypot(self.xy_nodes[best_nid][0] - x, self.xy_nodes[best_nid][1] - y)
        return best_nid, dist

    def snap_to_road_directed(self, x, y, yaw):
        """
        Finds closest road segment to (x, y) and determines forward departure node
        based on vehicle heading vector (cos(yaw), sin(yaw)).
        Returns: (forward_nid, rear_nid, is_dead_end, dist_to_segment)
        """
        if not self.road_graph:
            nid, d = self.snap_to_road(x, y)
            return nid, nid, False, d

        hx, hy = math.cos(yaw), math.sin(yaw)
        best_edge = None
        best_dist = float('inf')

        for u in self.road_graph:
            ux, uy = self.xy_nodes[u]
            for v in self.road_graph[u]:
                if u > v:
                    continue
                vx, vy = self.xy_nodes[v]
                dx, dy = vx - ux, vy - uy
                seg_len_sq = dx * dx + dy * dy
                if seg_len_sq < 1e-4:
                    continue
                t = max(0.0, min(1.0, ((x - ux) * dx + (y - uy) * dy) / seg_len_sq))
                px, py = ux + t * dx, uy + t * dy
                d = math.hypot(x - px, y - py)
                if d < best_dist:
                    best_dist = d
                    best_edge = (u, v)

        if not best_edge:
            nid, d = self.snap_to_road(x, y)
            return nid, nid, False, d

        u, v = best_edge
        ux, uy = self.xy_nodes[u]
        vx, vy = self.xy_nodes[v]

        dot_u = (ux - x) * hx + (uy - y) * hy
        dot_v = (vx - x) * hx + (vy - y) * hy

        forward_nid = u if dot_u > dot_v else v
        rear_nid = v if dot_u > dot_v else u

        is_dead_end = (len(self.road_graph.get(forward_nid, {})) <= 1)
        return forward_nid, rear_nid, is_dead_end, best_dist

    def get_stop_road_node(self, stop_name):
        """Retrieve the road node ID for a named campus stop."""
        for key in self.stops:
            if key.lower() == stop_name.lower().strip():
                return self.stops[key]['snapped_road_node_id']
        return None
