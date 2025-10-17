#!/usr/bin/env python
import math
#-------------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------------
#                     Geodetic functions in Python as library
#-------------------------------------------------------------------------------------------
#   manteiner: Matteo Bresciani     email: matteo.bresciani@phd.unipi.it    date: 18/3/2020
#-------------------------------------------------------------------------------------------

#-------------------------------------------------------------------------------------------
# convert from geodetic 2D to Cartesian (North-East) 2D
#-------------------------------------------------------------------------------------------
def ll2ne(ll0,  ll):
  if len(ll0)==2 and len(ll)==2:
    lat0 = ll0[0]
    lon0 = ll0[1]
    lat  = ll[0]
    lon  = ll[1]

    lat  = lat  * math.pi/180
    lon  = lon  * math.pi/180
    lat0 = lat0 * math.pi/180
    lon0 = lon0 * math.pi/180

    dlat = lat - lat0
    dlon = lon - lon0

    a  = 6378137.0
    f  = 1 / 298.257223563
    Rn = a / math.sqrt(1 - (2 * f - f * f) * math.sin(lat0) * math.sin(lat0))
    Rm = Rn * (1 - (2 * f - f * f)) / (1 - (2 * f - f * f) * math.sin(lat0) * math.sin(lat0))

    n = dlat / math.atan2(1, Rm)
    e = dlon / math.atan2(1, Rn * math.cos(lat0))
    ne = [n,e]
    return ne

  else:
    print('[gnc_utilities.py - ll2ne]: Error in the size of the inputs')
    return False  

#-------------------------------------------------------------------------------------------
# convert from Cartesian (North-East) 2D to geodetic 2D
#-------------------------------------------------------------------------------------------
def ne2ll(ll0, ne):  
  if len(ll0)==2 and len(ne)==2:
    lat0 = ll0[0]
    lon0 = ll0[1]
    lat0 = lat0 * math.pi/180
    lon0 = lon0 * math.pi/180

    a  = 6378137.0
    f  = 1 / 298.257223563
    Rn = a / math.sqrt(1 - (2 * f - f * f) * math.sin(lat0) * math.sin(lat0))
    Rm = Rn * (1 - (2 * f - f * f)) / (1 - (2 * f - f * f) * math.sin(lat0) * math.sin(lat0))

    lat = (lat0 + math.atan2(1, Rm)*ne[0]) * 180/math.pi
    lon = (lon0 + math.atan2(1, Rn*math.cos(lat0)) * ne[1]) * 180/math.pi
    ll  = [lat, lon] 
    return ll

  else:
    print('[gnc_utilities.py - ne2ll]: Error in the size of the inputs')
    return False

#-------------------------------------------------------------------------------------------
# convert from geodetic 3D to Cartesian (North-East) 3D
#-------------------------------------------------------------------------------------------
def lld2ned(lld0, lld):
  if len(lld0)==3 and len(lld)==3:
    ne  = ll2ne([lld0[0], lld0[1]], [lld[0], lld[1]])
    d   = lld[2] - lld0[2]
    ned = [ne[0], ne[1], d]
    return ned 
  else:
    print('[gnc_utilities.py - lld2ned]: Error in the size of the inputs')
    return False

#-------------------------------------------------------------------------------------------
# convert from Cartesian (North-East-Down) 3D to geodetic 3D
#-------------------------------------------------------------------------------------------
def ned2lld(lld0, ned):
  if len(lld0)==3 and len(ned)==3:
    ll  = ne2ll([lld0[0], lld0[1]], [ned[0], ned[1]])
    d   = ned[2] + lld0[2]
    lld = [ll[0], ll[1], d]
    return lld 
  else:
    print('[gnc_utilities.py - lld2ned]: Error in the size of the inputs')
    return False

#-------------------------------------------------------------------------------------------
# Given 2 geodetic 3D positions give back the DIRECTION between the two
#-------------------------------------------------------------------------------------------
def lld2direction(lld0, lld):
  if (len(lld0)==3 and len(lld)==3):
    tmp_ned   = lld2ned(lld0, lld)
    direction = math.atan2(tmp_ned[1],tmp_ned[0])
    return direction
  else:
    print('[gnc_utilities.py - lld2direction]: Error in the size of the inputs')
    return False

#-------------------------------------------------------------------------------------------
# Given 2 geodetic 2D positions give back the DIRECTION between the two
#-------------------------------------------------------------------------------------------
def ll2direction(ll0, ll):
  if (len(ll0)==2 and len(ll)==2):
    tmp_ned   = ll2ned(ll0, ll)
    direction = math.atan2(tmp_ned[1],tmp_ned[0])
    return direction
  else:
    print('[gnc_utilities.py - ll2direction]: Error in the size of the inputs')
    return False
#-------------------------------------------------------------------------------------------
# Given 2 geodetic 3D positions give back the DISTANCE between the two
#-------------------------------------------------------------------------------------------
def lld2distance(lld0, lld):
  if (len(lld0)==3 and len(lld)==3) or ((len(lld0)==2 and len(lld)==2)):
    tmp_ned  = lld2ned(lld0, lld)
    distance = math.sqrt(math.pow(tmp_ned[1],2) + math.pow(tmp_ned[0],2))
    return distance
  else:
    print('[gnc_utilities.py - lld2distance]: Error in the size of the inputs')
    return False

#-------------------------------------------------------------------------------------------
# Given 2 geodetic 2D positions give back the DISTANCE between the two
#-------------------------------------------------------------------------------------------
def ll2distance(ll0, ll):
  if ((len(ll0)==2 and len(ll)==2)):
    tmp_ned  = ll2ned(ll0, ll)
    distance = math.sqrt(math.pow(tmp_ned[1],2) + math.pow(tmp_ned[0],2))
    return distance
  else:
    print('[gnc_utilities.py - lld2distance]: Error in the size of the inputs')
    return False



