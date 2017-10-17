# -*- coding: utf-8 -*-
"""


@author: Benjamin Mathon

--------------------------------------------------------------------------
Acquisition sur camera IDS uEye, projet encadre IVI
Copyright (C) 2017  Universite Lille 1

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
--------------------------------------------------------------------------
"""

from OSC import OSCClient, OSCMessage, OSCBundle
import time
import numpy as np
import sys


class TuioServer:
    def __init__(self, address="localhost", port=3333):
        self.address = "localhost"
        self.port = 3333
        self.client = OSCClient()
        self.client.connect((address,port))
        self.fseqCount = -1
        self.alive = []
        
    
    def addExternalTuioCursor(self,cursor):
        self.alive.append(cursor)

    def getTuioCursor(self,sid):
        for cur in self.alive:
            if cur.sid == sid:
                return cur
        raise ValueError("Curseur non existant dans la liste.")

    def updateTuioCursor(self,cur,new_x,new_y):
        i = 0
        timestamp = time.time()

        while i < len(self.alive) and self.alive[i].sid != cur.sid:
            i += 1
        if i != len(self.alive):
            dt = timestamp - cur.timestamp
            dx = new_x - cur.x
            dy = new_y - cur.y

            #to avoid division by zero
            if dt < 1e-15:
                dt = 1e-15

            x_speed = dx/dt
            y_speed = dy/dt
            dist = np.sqrt(dx * dx + dy * dy)
            last_motion_speed = cur.motion_speed
            motion_speed = dist/dt
            motion_accel = (motion_speed - last_motion_speed)/dt

            self.alive[i].x = new_x
            self.alive[i].y = new_y
            self.alive[i].X = x_speed
            self.alive[i].Y = y_speed
            self.alive[i].motion_speed = motion_speed
            self.alive[i].m = motion_accel
            self.alive[i].timestamp = timestamp
        else:
            raise ValueError("Curseur non existant dans la liste.")

        
    def removeAllTuioCursor(self):
        self.alive = []
    
    def removeExternalTuioCursor(self,cur):
        i = 0
        timestamp = time.time()

        while i < len(self.alive) and self.alive[i].sid != cur.sid:
            i += 1
        if i != len(self.alive):
            self.alive.remove(self.alive[i])
        else:
            raise ValueError("Curseur non existant dans la liste.")
    
    def commitFrame(self):
        """
        A typical TUIO bundle will contain an initial ALIVE message, followed by an arbitrary number of SET messages
        that can fit into the actual bundle capacity and a concluding FSEQ message. A minimal TUIO bundle needs to
        contain at least the compulsory ALIVE and FSEQ messages. The FSEQ frame ID is incremented for each delivered
        bundle, while redundant bundles can be marked using the frame sequence ID -1.
        /tuio/2Dcur alive s_id0...s_idN
        /tuio/2Dcur set s_id x_pos y_pos x_vel y_vel m_accel
        /tuio/2Dcur fseq f_id
        """
        bundle = OSCBundle()
        if len(self.alive) == 0:
            aliveMsg = OSCMessage("/tuio/2Dcur")
            aliveMsg.append('alive')
            bundle.append(aliveMsg)
        else:
            aliveMsg = OSCMessage("/tuio/2Dcur")
            aliveMsg.append('alive')
            for cursor in self.alive:
                aliveMsg.append(cursor.sid)
            bundle.append(aliveMsg)
            for cursor in self.alive:
                msg = OSCMessage()

                # TUIO message: /tuio/2Dcur set s x y X Y m
                # s: Session ID (temporary object ID) (int32)
                # x, y: Position (float32)
                # X, Y: Velocity vector (motion speed & direction) (float32)
                # m: Motion acceleration (float32)

                msg.setAddress("/tuio/2Dcur")
                msg.extend(['set', cursor.sid, cursor.x, cursor.y, cursor.X, cursor.Y, cursor.m])

                bundle.append(msg)


        frameMsg = OSCMessage("/tuio/2Dcur")
        frameMsg.append('fseq')
        frameMsg.append(self.fseqCount)

        bundle.append(frameMsg)
        self.client.send(bundle)
        self.fseqCount = (self.fseqCount+1)%sys.maxint
        

class TuioCursor:
    def __init__(self, sid, x, y):
        self.sid = sid
        self.x   = x
        self.y   = y
        self.X   = 0.
        self.Y   = 0.
        self.m   = 0.
        self.timestamp = time.time()
        self.motion_speed = 0.
    
    def __repr__(self):
        #return "ID: %d x: %lf y: %lf speed: %lf acceleration: %lf timeStamp: %lf" % (self.sid, self.x, self.y, self.motion_speed, self.m, self.timestamp)
        return "%d" % self.sid