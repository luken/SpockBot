import collections
import logging

from spockbot.plugins.base import PluginBase, pl_announce
from spockbot.plugins.tools.collision import MTVTest, uncenter_position, center_position
from spockbot.mcdata import blocks
from spockbot.mcdata.utils import BoundingBox
from spockbot.vector import Vector3

"""
I wrote a lot of comments explaining whats broken and whats not but
they got lost somewhere in git. So just assume everything is pretty
crappy. Example plugin at:
https://gist.github.com/nickelpro/098e7158019365f67de6
"""

logger = logging.getLogger('spockbot')

class PathCore(object):
    def __init__(self, plug):
        self.plug = plug
        self.find_valid_nodes = plug.find_valid_nodes

    def pathfind(self, pos, target):
        pos = center_position(pos.floor(), BoundingBox(1, 1))
        target = center_position(target.floor(), BoundingBox(1, 1))

        return self.plug.pathfind(PathNode(pos), PathNode(target))

    def build_list_from_node(self, node):
        ret = collections.deque()
        ret.append(node)
        while ret[0].parent is not None:
            ret.appendleft(ret[0].parent)
        return ret

class PathNode(Vector3):
    def __init__(self, *xyz):
        super(PathNode, self).__init__(*xyz)
        self.parent = None
        self.child = None
        self.node_dist = 0
        self.is_jump = False
        self.is_fall = False

@pl_announce('Path')
class PathPlugin(PluginBase):
    requires = ('World', 'Physics', 'ClientInfo')

    def __init__(self, ploader, settings):
        super(PathPlugin, self).__init__(ploader, settings)

        self.bounding_box = BoundingBox(w=0.6, h=1.8)
        self.path = PathCore(self)
        self.col = MTVTest(self.world, self.bounding_box)
        ploader.provides('Path', self.path)

    def pathfind(self, start_node, end_node):
        def calc_f_val(node):
            return node.node_dist + end_node.dist(node)
        open_list = []
        open_list_preferred = []
        closed_list = []
        open_list_preferred.append(start_node)
        while open_list_preferred:
            current_node = open_list_preferred.pop(0)
            p = current_node.parent
            # Cut out nodes in between leaving only ends of a straight line
            # but jump nodes and fall nodes should always be individual
            if p is not None and not p.is_jump and not p.is_fall:
                pp = current_node.parent.parent
                if pp is not None and self.raycast_bbox(pp, current_node):
                    current_node.parent = pp
                    current_node.node_dist = pp.node_dist + current_node.dist(p)
            # XXX This test can fail badly as it ignores y
            if current_node - Vector3(0, current_node.y, 0) == end_node - Vector3(0, end_node.y, 0):
                return current_node
            test_walk, test_fall, test_jump = self.find_valid_nodes(current_node)
            moveable_nodes = []
            for valid_node in test_walk + test_fall + test_jump:
                if valid_node not in (open_list + closed_list + open_list_preferred):
                    moveable_nodes.append(valid_node)
            if moveable_nodes:
                moveable_nodes.sort(key=end_node.dist)
                open_list_preferred.append(moveable_nodes.pop(0))
                open_list += moveable_nodes

            if not open_list_preferred and open_list:
                # XXX Instead of falling back to the next closest option over and over
                # probably better to find the obstical edge and follow it in both directions
                # till <something smart> then we find a path around that way.
                open_list.sort(key=calc_f_val)
                open_list_preferred.append(open_list.pop(0))
            closed_list.append(current_node)
        return None

    def raycast_bbox(self, start, end):
        pos = PathNode(start)
        pos.x -= self.col.bounding_box.w/2
        pos.z -= self.col.bounding_box.d/2
        path = end - start
        if not path:
            return True
        depth = Vector3(*map(lambda a: a[0] * a[1], zip(path.norm(), self.bounding_box)))
        i, r = divmod(path.dist(), depth.dist())
        for j in range(int(i)):
            pos += depth
            block_pos = pos.floor()
            block_id, meta = self.world.get_block(
                block_pos.x, block_pos.y-1, block_pos.z
            )
            block = blocks.get_block(block_id, meta)
            if not block.bounding_box:
                return False
            t = self.col.block_collision(pos)
            if any(t):
                return False
        return True

    def get_block(self, pos):
        block_id, meta = self.world.get_block(*pos.vector)
        return blocks.get_block(block_id, meta)

    def check_for_bbox(self, pos):
        pos = pos.floor()
        block = self.get_block(pos)
        if block.bounding_box:
            return True
        block = self.get_block(pos + Vector3(0, 1, 0))
        if block.bounding_box:
            return True
        return False

    def single_query(self, node, offset, walk_nodes, fall_nodes, jump_nodes):
        walk_bool = fall_bool = jump_bool = False
        walk_node = node + offset
        if not self.check_for_bbox(walk_node):
            fall_node = node - Vector3(0, 1, 0)
            if not self.check_for_bbox(fall_node):
                fall_node.parent = node
                fall_node.node_dist = node.node_dist + fall_node.dist(node)
                fall_node.is_fall = True
                fall_nodes.append(fall_node)
                fall_bool = True
            else:
                walk_node.parent = node
                walk_node.node_dist = node.node_dist + walk_node.dist(node)
                walk_nodes.append(walk_node)
                walk_bool = True
        else:
            # Only test jump if walk and fall failed
            jump_node = walk_node + Vector3(0, 1, 0)
            if not self.check_for_bbox(jump_node):
                jump_node.parent = node
                jump_node.node_dist = node.node_dist + jump_node.dist(node)
                jump_node.is_jump = True
                jump_nodes.append(jump_node)
                jump_bool = True
        return walk_bool, fall_bool, jump_bool

    def find_valid_nodes(self, node):
        root_node = node
        walk_nodes, fall_nodes, jump_nodes = [], [], []
        pos_walk_x, pos_fall_x, pos_jump_x = self.single_query(root_node, Vector3(1, 0, 0), walk_nodes, fall_nodes, jump_nodes)
        pos_walk_z, pos_fall_z, pos_jump_z = self.single_query(root_node, Vector3(0, 0, 1), walk_nodes, fall_nodes, jump_nodes)
        neg_walk_x, neg_fall_x, neg_jump_x = self.single_query(root_node, Vector3(-1, 0, 0), walk_nodes, fall_nodes, jump_nodes)
        neg_walk_z, neg_fall_z, neg_jump_z = self.single_query(root_node, Vector3(0, 0, -1), walk_nodes, fall_nodes, jump_nodes)

        diag_nodes = []
        diag_nodes.append(walk_nodes if pos_walk_x and pos_walk_z else [])
        if (pos_walk_x or pos_fall_x) and (pos_walk_z or pos_fall_z):
            diag_nodes.append(fall_nodes)
        else:
            diag_nodes.append([])
        diag_nodes.append(jump_nodes if pos_jump_x and pos_jump_z else [])
        if any(diag_nodes):
            self.single_query(root_node, Vector3(1, 0, 1), *diag_nodes)

        diag_nodes = []
        diag_nodes.append(walk_nodes if neg_walk_x and neg_walk_z else [])
        if (neg_walk_x or neg_fall_x) and (neg_walk_z or neg_fall_z):
            diag_nodes.append(fall_nodes)
        else:
            diag_nodes.append([])
        diag_nodes.append(jump_nodes if neg_jump_x and neg_jump_z else [])
        if any(diag_nodes):
            self.single_query(root_node, Vector3(-1, 0, -1), *diag_nodes)

        diag_nodes = []
        diag_nodes.append(walk_nodes if pos_walk_x and neg_walk_z else [])
        if (pos_walk_x or pos_fall_x) and (neg_walk_z or neg_fall_z):
            diag_nodes.append(fall_nodes)
        else:
            diag_nodes.append([])
        diag_nodes.append(jump_nodes if pos_jump_x and neg_jump_z else [])
        if any(diag_nodes):
            self.single_query(root_node, Vector3(1, 0, -1), *diag_nodes)

        diag_nodes = []
        diag_nodes.append(walk_nodes if neg_walk_x and pos_walk_z else [])
        if (neg_walk_x or neg_fall_x) and (pos_walk_z or pos_fall_z):
            diag_nodes.append(fall_nodes)
        else:
            diag_nodes.append([])
        diag_nodes.append(jump_nodes if neg_jump_x and pos_jump_z else [])
        if any(diag_nodes):
            self.single_query(root_node, Vector3(-1, 0, 1), *diag_nodes)

        return walk_nodes, fall_nodes, jump_nodes
