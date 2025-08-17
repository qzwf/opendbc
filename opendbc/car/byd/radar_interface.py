from opendbc.car import structs
RadarData = structs.RadarData
from opendbc.car.interfaces import RadarInterfaceBase


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    # BYD ATTO3 uses camera-based detection, no radar
    self.rcp = None
    self.pts = {}
    self.delay = 0
    self.radar_ts = 0

  def update(self, can_parsers):
    # BYD ATTO3 doesn't have radar, return empty radar data
    ret = RadarData()
    ret.errors = []
    ret.canMonoTimes = []

    # No radar points for camera-based system
    for i in range(16):  # Standard radar track count
      ret.points[i].trackId = i
      ret.points[i].dRel = 0.0
      ret.points[i].yRel = 0.0
      ret.points[i].vRel = 0.0
      ret.points[i].aRel = 0.0
      ret.points[i].yvRel = 0.0
      ret.points[i].measured = False

    return ret