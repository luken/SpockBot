from spockbot.plugins.core import auth, event, net, taskmanager, ticker, timers
from spockbot.plugins.helpers import channels, chat, clientinfo, craft, \
    entities, interact, inventory, keepalive, \
    movement, pathfinding, physics, respawn, start, world

core_plugins = [
    ('auth', auth.AuthPlugin),
    ('event', event.EventPlugin),
    ('net', net.NetPlugin),
    ('taskmanager', taskmanager.TaskManager),
    ('ticker', ticker.TickerPlugin),
    ('timers', timers.TimersPlugin),
]
helper_plugins = [
    ('channels', channels.ChannelsPlugin),
    ('chat', chat.ChatPlugin),
    ('clientinfo', clientinfo.ClientInfoPlugin),
    ('craft', craft.CraftPlugin),
    ('entities', entities.EntitiesPlugin),
    ('interact', interact.InteractPlugin),
    ('inventory', inventory.InventoryPlugin),
    ('keepalive', keepalive.KeepalivePlugin),
    ('movement', movement.MovementPlugin),
    ('pathfinding', pathfinding.PathfindingPlugin),
    ('physics', physics.PhysicsPlugin),
    ('respawn', respawn.RespawnPlugin),
    ('start', start.StartPlugin),
    ('world', world.WorldPlugin),
]
default_plugins = core_plugins + helper_plugins
