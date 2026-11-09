"""One owner for A→B→C. Sequential arm motion, persistent B reservation."""
from .workflow import Arm1Workflow
from .arm2_workflow import Arm2Workflow


class BReservation:
    def __init__(self, owner=''):
        if owner not in ('', 'arm1', 'arm2'):
            raise ValueError('Invalid B owner')
        self.owner = owner

    def acquire(self, arm):
        if arm not in ('arm1', 'arm2') or self.owner not in ('', arm):
            raise RuntimeError('B is reserved by ' + self.owner)
        self.owner = arm

    def release(self, arm, verified):
        if self.owner != arm or not verified:
            raise RuntimeError('B release requires current owner and verified departure')
        self.owner = ''


def run_transfer(io, limit):
    """At most limit objects; inspect C before taking another object from A.

    An object already in B is consumed first. No restart resumes this function.
    The IO adapter persists every arm transition before allowing the next move.
    """
    io.initialize()
    if not any(io.occupied()):
        zone = io.arm1.zone()
        if not zone.success or not zone.b_arm_clear or zone.c_object_count:
            raise RuntimeError('A ranking requires C empty and a clear B zone')
        if not zone.b_has_object:
            io.arm1.capture_ranking()
    for _ in range(limit):
        io.guard()
        occupied = io.occupied()
        io.arm2.verify_layout(occupied)
        if all(occupied):
            return 'FULL'
        zone = io.arm1.zone()
        if not zone.success or not zone.b_arm_clear or zone.c_object_count != sum(occupied):
            raise RuntimeError('B not clear or C observation inconsistent with journal')
        if not zone.b_has_object:
            Arm1Workflow(io.arm1, io.arm1.cfg, io.cancel,
                         lambda s: io.arm_state('arm1', s)).run()
        io.guard()
        # The two clients use one planning scene: the B object must be transferred
        # out of the world representation before arm2 approaches to grasp it.
        io.arm1.clear_placed_disk()
        Arm2Workflow(io.arm2, io.arm2.cfg, occupied, io.cancel,
                     lambda s: io.arm_state('arm2', s)).run()
        io.completed()
    return 'FULL' if all(io.occupied()) else 'SUCCEEDED'
