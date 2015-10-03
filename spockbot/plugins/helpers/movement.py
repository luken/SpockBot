"""
MovementPlugin provides a centralized plugin for controlling all outgoing
position packets so the client doesn't try to pull itself in a dozen
directions.
Also provides very basic pathfinding
"""

import logging

from spockbot.plugins.base import PluginBase, pl_announce
from spockbot.vector import Vector3

logger = logging.getLogger('spockbot')


class MovementCore(object):
    def __init__(self, plug):
        self.move_location = None
        self.plug = plug

    def move_to(self, x, y, z):
        logger.debug('move_to')
        self.move_location = Vector3(x, y, z)
        self.plug.setup_pathfinding()

    def stop(self):
        self.move_location = None
        self.plug.teardown_pathfinding()


    def is_moving(self):
        return self.move_location is not None


@pl_announce('Movement')
class MovementPlugin(PluginBase):
    requires = ('Net', 'Physics', 'ClientInfo', 'Event', 'Path')
    events = {
        'client_tick': 'client_tick',
        'action_tick': 'action_tick',
        'client_position_update': 'handle_position_update',
        'physics_collision': 'handle_collision',
        'client_join_game': 'handle_join_game',
    }

    def __init__(self, ploader, settings):
        super(MovementPlugin, self).__init__(ploader, settings)

        self.flag_pos_reset = False
        self.movement = MovementCore(self)
        self.connected_to_server = False
        ploader.provides('Movement', self.movement)
        self.path_nodes = None

    def client_tick(self, name, data):
        if not self.connected_to_server:
            return
        self.net.push_packet('PLAY>Player Position and Look',
                             self.clientinfo.position.get_dict())
        if self.flag_pos_reset:
            self.event.emit('movement_position_reset')
            self.flag_pos_reset = False

    def handle_join_game(self, name, data):
        self.connected_to_server = True

    def handle_position_update(self, name, data):
        self.flag_pos_reset = True

    def handle_collision(self, name, data):
        if self.movement.move_location is not None:
            self.physics.jump()

    def action_tick(self, name, data):
        self.do_pathfinding()

    def teardown_pathfinding(self):
        self.move_nodes = None

    def setup_pathfinding(self):
        logger.debug("setup_pathfinding")
        nodes = self.path.pathfind(self.clientinfo.position, self.movement.move_location)
        if nodes is None:
            logger.warn("Unable to find path from %s to %s, move command aborted" % (self.clientinfo.position, self.movement.move_location))
        else:
            logger.debug("pathfind returned nodes %s" % (nodes))
            self.path_nodes = self.path.build_list_from_node(nodes)

    def path_to(self):
        if not self.path_nodes:
            self.movement.stop()
            return
        n = self.path_nodes[0]
        if n.is_jump:
            logger.debug("path_to triggered jump")
            self.physics.jump()
        if self.physics.move_target(n):
            arrived = self.path_nodes.popleft()
            self.path_to()

    def do_pathfinding(self):
        if self.movement.move_location is not None:
            self.path_to()
