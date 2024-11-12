import threading
import copy
import time
import sys
import traceback
import pdb


import numpy as np

import matplotlib
from matplotlib import pyplot as pl

import pulseio
import pwmio
import digitalio

enable_wind=False
enable_vertical_motion=False

def subplotparams_from_axessubplot(axessubplot):
    """ Return the pl.subplot() parameters corresponding
    to a particular AxesSubplot object so we can 
    make it current again. call with
    pl.subplot(*subplotparams_from_axessubplot(...))
    """
    nrows = axessubplot.get_gridspec().nrows
    ncols = axessubplot.get_gridspec().ncols

    num1 = axessubplot.get_subplotspec().num1
    #num2 = axessubplot.get_subplotspec().num2

    index = 1 + num1
    return (nrows,ncols,index)

class dynamic_model_interactor(object):
    model = None
    timer = None
    fig = None
    arrowabove = None
    closing=None

    subfig_xy=None # figure representing looking down at the craft from above
    arrow_xy=None
    subfig_yz=None
    arrow_yz=None
    subfig_xz=None
    arrow_xz=None
    subfig_slider=None # figure with slider for rotational control
    slider=None # actual slider widget

    subfig_altitude=None # figure representing altitude
    altitude_plot=None
    subfig_altitude_slider=None # figure with slider for altitude control
    altitude_slider=None # altitude slider widget

    def get_pos_and_quat(self):
        with self.model.model_lock:
            quat=copy.deepcopy(self.model.quat)
            pos=copy.deepcopy(self.model.pos)
            pass
        return (pos,quat)

    
    def __init__(self,model):
        self.model=model
        self.arrow_xy=None
        self.arrow_yz = None
        self.arrow_xz=None
        self.closing=False
        pass

    def draw_arrows(self,position,orientation,world_vec_frontward):

        transformed_vec_frontward = vec_apply_rot(orientation,world_vec_frontward)
        
        #print("transformed",transformed_vec_frontward)

        if self.subfig_xy is not None:
            pl.subplot(*subplotparams_from_axessubplot(self.subfig_xy))
            if self.arrow_xy is not None:
                self.arrow_xy.remove()
                pass
            self.arrow_xy = pl.arrow(-0.5*transformed_vec_frontward[0],-0.5*transformed_vec_frontward[1],transformed_vec_frontward[0],transformed_vec_frontward[1],width=0.2)
            pass

        if self.subfig_yz is not None:
            pl.subplot(*subplotparams_from_axessubplot(self.subfig_yz))
            if self.arrow_yz is not None:
                self.arrow_yz.remove()
                pass
            self.arrow_yz = pl.arrow(-0.5*transformed_vec_frontward[1],-0.5*transformed_vec_frontward[2],transformed_vec_frontward[1],transformed_vec_frontward[2],width=0.2)
            #print("arrow:",-0.5*transformed_vec_frontward[1],-0.5*transformed_vec_frontward[2],world_vec_frontward[1],transformed_vec_frontward[2])
            pass

        if self.subfig_xz is not None:
            pl.subplot(*subplotparams_from_axessubplot(self.subfig_xz))
            if self.arrow_xz is not None:
                self.arrow_xz.remove()
                pass
            self.arrow_xz = pl.arrow(-0.5*transformed_vec_frontward[0],-0.5*transformed_vec_frontward[2],transformed_vec_frontward[0],transformed_vec_frontward[2],width=0.2)
            pass

        if self.subfig_altitude is not None:
            pl.subplot(*subplotparams_from_axessubplot(self.subfig_altitude))
            if self.altitude_plot is not None:
                # [ plot.get_axes().remove() for plot in self.altitude_plot]
                self.subfig_altitude.clear()
                pass
            self.altitude_plot = pl.plot([0,10],[position[2],position[2]])
            pl.axis([0,10,0,10])
            pl.grid(True)
            pass

        pass

    def interactor_update(self):
        # May ONLY be called from main thread... typically by the timer.

        if self.closing:
            return

        
        with self.model.model_lock:
            if self.slider is not None:
                self.model.ROTCMD = int(self.slider.val*1000)
                pass

            if self.altitude_slider is not None:
                self.model.LIFTCMD = int(self.altitude_slider.val*1000)
                pass
            pass

        
        (position, orientation) = self.get_pos_and_quat()

        self.draw_arrows(position,orientation,self.world_vec_frontward)
        if "LED" in digitalio.DigitalInOuts_by_pin:
            if digitalio.DigitalInOuts_by_pin["LED"].value:
                pl.title("Calibration LED on")
                pass
            else:
                pl.title("Calibration LED off")
                pass
            pass
        self.fig.canvas.draw()
        pass

    def close_handler(self,event):
        self.closing=True
        self.timer.stop()

        sys.exit(0)
        pass
    
    def interactor(self):
        # This MUST be called from the main thread because the main thread is the ONLY one allowed to do GUI/matplotlib stuff!

        try: 
            (position,orientation) = self.get_pos_and_quat()

            # Frontward direction is y
            self.world_vec_frontward=vec_apply_rot(orientation,np.array((0.0,1.0,0.0),dtype='d'))

            
            self.fig=pl.figure()
            self.fig.canvas.mpl_connect('close_event',self.close_handler)
            self.subfig_xy = pl.subplot(2,2,1)
            pl.axis((-2,2,-2,2))
            if enable_vertical_motion:
                self.subfig_slider = pl.subplot(2,2,2)
                self.subfig_altitude = pl.subplot(2,2,3)
                self.subfig_altitude_slider = pl.subplot(2,2,4)
                pass
            else:
                self.subfig_yz = pl.subplot(2,2,3)        
                pl.axis((-2,2,-2,2))
                self.subfig_xz = pl.subplot(2,2,2)
                pl.axis((-2,2,-2,2))
                self.subfig_slider = pl.subplot(2,2,4)
                pass
            self.draw_arrows(position,orientation,self.world_vec_frontward)

            if self.subfig_xy is not None:
                
                pl.subplot(*subplotparams_from_axessubplot(self.subfig_xy))
                pl.xlabel('x')
                pl.ylabel('y')
                pass
            
            if self.subfig_yz is not None:
                pl.subplot(*subplotparams_from_axessubplot(self.subfig_yz))
                pl.xlabel('y')
                pl.ylabel('z')
                pass
            
            if self.subfig_xz is not None:
                pl.subplot(*subplotparams_from_axessubplot(self.subfig_xz))
                pl.xlabel('x')
                pl.ylabel('z')
                pass

            if self.subfig_altitude is not None:
                pl.subplot(*subplotparams_from_axessubplot(self.subfig_altitude))
                #pl.xlabel('x')
                pl.ylabel('z')
                pass
            
            

            if self.subfig_slider is not None:
                slider_ax = pl.subplot(*subplotparams_from_axessubplot(self.subfig_slider))
                self.slider = matplotlib.widgets.Slider(slider_ax,'ROT CMD',1.0,2.0,valinit=1.5)
                pl.xlabel('Click above to adjust\nROT CMD pulse from 0.5 to 1.5 ms')
                #self.slider.on_changed(slider_changed)
                pass

            if self.subfig_altitude_slider is not None:
                altitude_slider_ax = pl.subplot(*subplotparams_from_axessubplot(self.subfig_altitude_slider))
                self.altitude_slider = matplotlib.widgets.Slider(altitude_slider_ax,'LIFT CMD',1.0,2.0,valinit=1.0)
                pl.xlabel('Click above to adjust\nLIFT CMD pulse from 0.5 to 1.5 ms')
                #self.slider.on_changed(slider_changed)
                pass
            
            
            self.timer=self.fig.canvas.new_timer(interval=100) # interval in ms
            self.timer.add_callback(self.interactor_update)
            self.timer.start()
            
            pl.show()
            pass
        except:
            (extype, value, tb) = sys.exc_info()
            traceback.print_exc()
            pdb.post_mortem(tb)
            pass
        
        pass

def quatinv(a):
    return np.array((a[0],-a[1],-a[2],-a[3]),dtype='d')


def quatmul(a,b):
    return np.array((a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
                     a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
                     a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
                     a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]),dtype='d')

    
    
def vec_apply_rot(rot,vec):
    # Apply rotation by conjugation operation p'=qpq^-1

    rotnorm = rot/np.linalg.norm(rot)
    vecquat = np.array((0.0,vec[0],vec[1],vec[2]),dtype='d')
    
    
    rotnorminv = np.array((rotnorm[0],-rotnorm[1],-rotnorm[2],-rotnorm[3]),dtype='d')
    
    rotvec = quatmul(quatmul(rotnorm,vecquat),rotnorminv)

    #print("rotnorm",rotnorm)
    #print("rotnorminv",rotnorminv)
    #print("vecquat",vecquat)
    #print("rotvec",rotvec)
    #assert(abs(rotvec[0]) < 1e-8*np.linalg.norm(rotvec[1:]))
    
    return rotvec[1:]

    
class dynamic_model(object):
    quat = None # orientation quaternion... array of (w,x,y,z)... should always be a unit quaternion... apply quat to a vector in the object frame to get a vector in the world frame.
    pos=None # position offset, array of (x,y,z)
    
    angularvel = None # angular velocities ... array of (vx,vy,vz) in rad/s
    vel=None # linear velocity array in m/s
    mass = None  # Mass in kg
    balloon_lift = None # lift force in newtons
    Icm = None # Moment of inertia tensor about the center of mass, in object coordinates
    Icm_inverse = None # Inverse of Icm matrix
    ROTCMD = None # rotation command, in us (1500 nominal)
    LIFTCMD=None # lift/altitude command, in us (1000 nominal)
    
    interactor = None

    model_thread = None
    model_lock = None
    model_start_time=None
    
    time_step = None # seconds
    
    def __init__(self):

        # Define the initial state of the model
        
        self.quat=np.array((1.0,0.0,0.0,0.0),dtype='d') # null rotation
        self.pos = np.array((0.0,0.0,0.0),dtype='d') # start at origin
        self.angularvel = np.array((0.0,0.0,0.0),dtype='d') # Zero angular velocity
        self.vel = np.array((0.0,0.0,0.0),dtype='d') # initialize not moving
        self.mass = 0.5 # Mass of free body, kg
        self.balloon_lift = self.mass*9.8*0.85 # give lift that is 85% of the total weight

        # This is the initial assumed value of
        # the rotation command that you can apply through
        # clicking in the lower-right plot
        self.ROTCMD=1500
        self.LIFTCMD=1000

        # The radius of gyration in each axis is
        # a measure of how distributed the mass is
        # away from the center
        radius_of_gyration_x = 0.1 # meters
        radius_of_gyration_y = 0.1 # meters
        radius_of_gyration_z = 0.1 # meters

        # Assuming (x,y,z) align with the natural frame
        # of the object, the moment of inertia tensor
        # is diagonal with elements corresponding to
        # the mass times the radius of gyration around
        # each axis.
        
        self.Icm = np.diag((self.mass*radius_of_gyration_x,self.mass*radius_of_gyration_y,self.mass*radius_of_gyration_z))


        # We use locking to keep the different threads
        # from interfering with each other. This next line
        # just initializes the lock object. 
        self.model_lock=threading.Lock()
        self.model_start_time=time.monotonic()


        
        self.time_step = .2 # Time step per loop of the model_iteration function below
        
        self.thruster_force_per_ms = 5.0 # Newtons.... 5 N is a little more than 1 pound
        self.thruster_dist_from_cm = 0.2 # Distance of the thusters from the center of mass

        self.interactor=dynamic_model_interactor(self)
        self.model_thread = threading.Thread(target=self.model_iteration)
        self.model_thread.start()
        pass

    def model_iteration(self):
        
        last_time = 0.0
        itercnt = 1
        while True:
            try:
                target_time = last_time + self.time_step
                curtime=time.monotonic()-self.model_start_time
                #print("target_time",target_time)
                #print("curtime",curtime)

                # if it hasn't been long enough since our
                # last time step 
                if target_time > curtime:
                    time.sleep(target_time-curtime)
                    pass

                # Store the actual time for this step
                curtime=time.monotonic()-self.model_start_time

                # ... and determine how big the delta-t is since
                # the last time step 
                this_time_step = curtime - last_time
                
                last_time = curtime # store current time to use as last_time for next iteration

                # Acquire the model lock so that data 
                # changes atomically as seen from
                # other threads that also use the
                # model_lock
                with self.model_lock:

                    # Extract the commanded PWM output pulse 
                    # width from the pin assigned to "LEFT OUT"
                    # in board.py and store this in the
                    # variable left_pulse_width_sec
                    if "LEFT OUT" in pwmio.PWMOuts_by_pin:
                        
                        LEFT_PWM = pwmio.PWMOuts_by_pin["LEFT OUT"]
                        left_pulse_width_sec = (1.0/LEFT_PWM.frequency)*(LEFT_PWM.duty_cycle/65536.0)
                        
                        left_gap_width_sec = (1.0/LEFT_PWM.frequency)*((65536.0-LEFT_PWM.duty_cycle)/65536.0)
                        if left_pulse_width_sec < .99e-3 or left_pulse_width_sec > 2.01e-3:
                            raise ValueError("Invalid LEFT OUT pulse width of %f ms" % (left_pulse_width_sec*1000.0))
                        
                        if left_gap_width_sec < 15e-3 or left_gap_width_sec > 120e-3:
                            raise ValueError("Invalid LEFT OUT gap width of %f ms" % (left_gap_width_sec*1000.0))
                        pass
                    else:
                        left_pulse_width_sec=1.5e-3
                        pass
                    
                    # Extract the commanded PWM output pulse 
                    # width from the pin assigned to "RIGHT OUT"
                    # in board.py and store this in the
                    # variable right_pulse_width_sec
                    if "RIGHT OUT" in pwmio.PWMOuts_by_pin:
                        RIGHT_PWM = pwmio.PWMOuts_by_pin["RIGHT OUT"]
                        right_pulse_width_sec = (1.0/RIGHT_PWM.frequency)*(RIGHT_PWM.duty_cycle/65536.0)
                        
                        right_gap_width_sec = (1.0/RIGHT_PWM.frequency)*((65536.0-RIGHT_PWM.duty_cycle)/65536.0)
                        if right_pulse_width_sec < .99e-3 or right_pulse_width_sec > 2.01e-3:
                            raise ValueError("Invalid RIGHT OUT pulse width of %f ms" % (right_pulse_width_sec*1000.0))
                        
                        if right_gap_width_sec < 15e-3 or right_gap_width_sec > 120e-3:
                            raise ValueError("Invalid RIGHT OUT gap width of %f ms" % (right_gap_width_sec*1000.0))
                        pass
                    else:
                        right_pulse_width_sec = 1.5e-3
                        pass
                    # print("right_pulse_width_ms = %f" % (right_pulse_width_sec*1e3))

                    # Extract the commanded PWM output pulse 
                    # width from the pin assigned to "LIFT OUT"
                    # in board.py and store this in the
                    # variable lift_pulse_width_sec
                    if "LIFT OUT" in pwmio.PWMOuts_by_pin:
                        LIFT_PWM = pwmio.PWMOuts_by_pin["LIFT OUT"]
                        lift_pulse_width_sec = (1.0/LIFT_PWM.frequency)*(LIFT_PWM.duty_cycle/65536.0)
                        
                        lift_gap_width_sec = (1.0/LIFT_PWM.frequency)*((65536.0-LIFT_PWM.duty_cycle)/65536.0)
                        if lift_pulse_width_sec < .99e-3 or lift_pulse_width_sec > 2.01e-3:
                            raise ValueError("Invalid LIFT OUT pulse width of %f ms" % (lift_pulse_width_sec*1000.0))
                        
                        if lift_gap_width_sec < 15e-3 or lift_gap_width_sec > 120e-3:
                            raise ValueError("Invalid LIFT OUT gap width of %f ms" % (lift_gap_width_sec*1000.0))
                        pass
                    else:
                        lift_pulse_width_sec = 1.0e-3
                        pass
                    # print("lift_pulse_width_ms = %f" % (lift_pulse_width_sec*1e3))


                    # Add a moment about z due to wind if wind enabled
                    if enable_wind:
                        self.wind_moment = 0.05 # N*m
                        pass
                    else:
                        self.wind_moment = 0.0
                        pass

                    # Dynamic model
                    # -------------
                    
                    # Calculate sum of moments on the craft about the
                    # center of mass, in craft coordinates
                    # This will be stored in the variable
                    # Hdot_cm_craft_coords, representing the
                    # instantaneous change in angular momentum

                    Hdot_cm_craft_coords_x = 0.0
                    Hdot_cm_craft_coords_y = 0.0

                    # if we are looking from above with
                    # x to our right, y forward and 
                    # the z axis coming toward us, rotation
                    # is positive counter-clockwise by the
                    # right-hand rule.
                    #
                    # Thus the left thruster, which would make
                    # us rotate clockwise, gives a negative
                    # moment with a pulse width greater than 1.5 ms

                    # Convert from pulse width to newtons by the
                    # thruster_force_per_ms scaling factor and
                    # obtain a moment (torque) by multiplying
                    # by the lever arm of the thruster from
                    # the center of mass 

                    left_thruster_moment = -(left_pulse_width_sec*1e3-1.5)*self.thruster_force_per_ms*self.thruster_dist_from_cm

                    left_thruster_force = ((left_pulse_width_sec*1e3-1.5)*self.thruster_force_per_ms) *np.array((0,1,0)) # force in the forward (+y) direction
                    
                    # Right thruster is the same except positive
                    # settings makes us rotate CCW which
                    # is a positive rotation around the Z axis. 
                    right_thruster_moment = (right_pulse_width_sec*1e3-1.5)*self.thruster_force_per_ms*self.thruster_dist_from_cm

                    right_thruster_force = ((right_pulse_width_sec*1e3-1.5)*self.thruster_force_per_ms) *np.array((0,1,0)) # force in the forward (+y) direction

                    lift_thruster_force = ((lift_pulse_width_sec*1e3-1.5)*self.thruster_force_per_ms) *np.array((0,0,1)) # force in the up (+z) direction

                    # Treat wind damping as a drag torque
                    # opposing the angular velocity
                    wind_damping_factor = -.02 # N*m / (rad/s)  # Include a very small amount of damping

                    # Treat air drag as a force opposing motion
                    # Dynamic pressure q
                    rho = 1.2 #kg/m^3
                    vel_magnitude = np.linalg.norm(self.vel)
                    q = 0.5*rho*vel_magnitude**2
                    SCd = 0.2 #area*drag coefficient, m^2
                    # Drag force D in direction opposing velocity
                    if vel_magnitude != 0.0:
                        D = -q*SCd*(self.vel/vel_magnitude)
                        pass
                    else:
                        D = 0.0
                        pass
                    

                    # self.imu_gyro[2] gives the rotation rate around the
                    # craft z axis in rad/s
                    # (if we had motion around other axes we should
                    # probably include these in the sum of moments
                    # as well)
                    wind_damping_z = self.imu_gyro[2]*wind_damping_factor
                    # F = ma -> a = F/m
                    # Transform thruster forces into world coordinates
                    left_thruster_force_worldcoords = vec_apply_rot(self.quat,left_thruster_force)
                    right_thruster_force_worldcoords = vec_apply_rot(self.quat,right_thruster_force)
                    lift_thruster_force_worldcoords = vec_apply_rot(self.quat,lift_thruster_force)
                    # Newton's law says sum of forces equals derivative of linear momentum
                    Ldot  = (left_thruster_force_worldcoords + right_thruster_force_worldcoords + lift_thruster_force_worldcoords + D)

                   
                    # Newton's law says sum of moments = change in
                    # angular momentum Hdot
                    
                    Hdot_cm_craft_coords_z = left_thruster_moment + right_thruster_moment + wind_damping_z + self.wind_moment

                    # Create numpy array representing the change in
                    # angular momentum 
                    Hdot_cm_craft_coords = np.array((Hdot_cm_craft_coords_x,
                                                     Hdot_cm_craft_coords_y,
                                                     Hdot_cm_craft_coords_z),dtype='d') 
                    
                    # Transform Hdot into world coordinates by applying
                    # the self.quat transformation to the Hdot vector,
                    # obtaining the change in angular momentum about
                    # the object's center of mass as seen
                    # in world (X,Y,Z) cordinates instead of
                    # the object's frame. 

                    #print("Hdot_cm_craft_coords",Hdot_cm_craft_coords,((right_pulse_width_sec*1e3-1.5)-(left_pulse_width_sec*1e3-1.5))*self.thruster_force_per_ms*self.thruster_dist_from_cm)
                    
                    Hdot_cm=vec_apply_rot(self.quat,Hdot_cm_craft_coords)

                    # Update method calculates new
                    # object orientation given a particular
                    # change in angular momentum
                    self.update(Ldot,Hdot_cm,this_time_step)

                    # Pass slider setting to PulseIn
                    
                    #print("Adding pulse: %d" % (int(self.ROTCMD)))
                    if "ROT CMD" in pulseio.PulseIns_by_pin:
                        pulseio.PulseIns_by_pin["ROT CMD"]._Add_pulse(int(self.ROTCMD),True)
                        pulseio.PulseIns_by_pin["ROT CMD"]._Add_pulse(20000,False)
                        pass

                    if "LIFT CMD" in pulseio.PulseIns_by_pin:
                        pulseio.PulseIns_by_pin["LIFT CMD"]._Add_pulse(int(self.LIFTCMD),True)
                        pulseio.PulseIns_by_pin["LIFT CMD"]._Add_pulse(20000,False)
                        pass

                    # FWD CMD hard-wired to 1500us neutral position
                    if "FWD CMD" in pulseio.PulseIns_by_pin:
                        pulseio.PulseIns_by_pin["FWD CMD"]._Add_pulse(1500,True)
                        pulseio.PulseIns_by_pin["FWD CMD"]._Add_pulse(20000,False)
                        pass

                    pass
                pass
            except:
                
                (extype, value, tb) = sys.exc_info()
                traceback.print_exc()
                pdb.post_mortem(tb)
                pass

            itercnt +=1 
            pass
        

    def update(self,Ldot,Hdot_cm,dt):
        # Model_lock should be locked when making this call
        # Hdot_cm is the sum of moments, in world coordinates, about the center of mass

        # Update velocity
        self.vel += Ldot*dt
        # Update position
        self.pos += self.vel*dt

        # put a floor under it:
        if self.pos[2] < 0:
            self.pos[2] = 0
            self.vel[2] = -self.vel[2] # bouncy floor
            pass
        # Update angular velocity
        # Based on Ruina, chapter 8, pg 480
        # Hcm = [Icm] dot omega

        # We need to convert Icm, the moment of inertia
        # tensor about the object's center of mass,
        # into fixed world (X,Y,Z) components from
        # object components

        # In order to do this, we have to apply the
        # transformation to the left (columns) as well
        # as the right (rows)
        
        # This makes the Icm we actually use dependent
        # on the current rotation.

        # Apply quat to columns...
        Icm0 = np.zeros((3,3),dtype='d') # temporary
        for col in range(3):
            Icm0[:,col]=vec_apply_rot(self.quat,self.Icm[:,col])
            pass
        # Apply quat inverse to rows to get Icm in world coordinates
        Icm = np.zeros((3,3),dtype='d') 
        qi = quatinv(self.quat)
        for row in range(3):
            Icm[row,:]=vec_apply_rot(qi,Icm0[row,:])
            pass
        
        Icm_inverse = np.linalg.inv(Icm)

        # Angular momentum is the tensor product
        # of the moment of inertia tensor with the angular velocity
        H_cm = np.dot(Icm,self.angularvel)

        # Derivative of the angular momentum: 
        # Hdot_cm = [Icm] times omegadot + omega cross H_cm
        # where Hdot_cm can be calculated from the sum of moments...
        # so we can solve for omegadot:
        Icm_times_omegadot = Hdot_cm - np.cross(self.angularvel,H_cm) 
        omegadot = np.dot(Icm_inverse,Icm_times_omegadot)

        #print ("angularvel",self.angularvel)
        #print("omegadot",omegadot)
        #print("dt",dt)
        #print("Hdot_cm",Hdot_cm)

        # For this timestep, increment angular velocity
        # according to the calculated omegadot and the
        # time step. 
        self.angularvel += omegadot * dt
        
        # Update orientation according to angular velocity
        # ... This assumes that the rotation in a single
        # step is not too large.

        # Start with our current orientation, represented
        # in quaternion form 
        quat = self.quat
        
        for axisidx in range(3):

            # In each axis, represent the rotation around
            # that axis in quaternion form. 
            rotvel=self.angularvel[axisidx]
            rotmag = rotvel * dt
            rotquat = np.zeros(4,dtype='d')
            rotquat[axisidx+1] = np.sin(rotmag/2.0)
            rotquat[0] = np.cos(rotmag/2.0)
            rotquat /= np.linalg.norm(rotquat) # normalize quaternion

            # multiply this rotation into our quaternion
            quat=quatmul(quat,rotquat)

            # re-normalize our quaternion
            quat=quat/np.linalg.norm(quat)
            
            pass
        
        #previous_euler = self.imu_euler

        # Store quaternion as our new orientation. 
        self.quat=quat

        #print("Angular velocity = %s rad/s" % (self.angularvel))
        
        new_euler = self.imu_euler
        #print("Rotation from %s to %s" % (str(previous_euler),str(new_euler)))
        pass
    
            

    @property
    def imu_calibrated(self):
        # Auto-calibrate after 30 seconds
        if time.monotonic()-self.model_start_time >= 30.0:
            return True
        return False
    
    @property
    def imu_matrix(self):
        # Get matrix representing our orientation
        quat=self.quat
        w=0
        x=1
        y=2
        z=3
        
        return np.array(((1 - 2*quat[y]**2 - 2*quat[z]**2, 2*quat[x]*quat[y] - 2*quat[w]*quat[z], 2.0*quat[x]*quat[z] + 2*quat[w]*quat[y]),
                         (2.0*quat[x]*quat[y] + 2.0*quat[w]*quat[z], 1.0 - 2*quat[x]**2 - 2.0*quat[z]**2, 2.0*quat[y]*quat[z] - 2.0*quat[w]*quat[x]),
                          (2.0*quat[x]*quat[z] - 2.0*quat[w]*quat[y], 2.0*quat[y]*quat[z] + 2*quat[w]*quat[x], 1.0 - 2*quat[x]**2 - 2.0*quat[y]**2)),dtype='d') 
    
    @property
    def imu_euler(self):
        # Convert our quaternion into Euler angles (heading, roll, pitch)
        
        # Note: Rotation order is roll about y -> pitch about x -> yaw about z
        # imu_euler[0] is heading (yaw)
        # imu_euler[1] is roll
        # imu_euler[2] is pitch

        # NOTE: Not sure this accurately represents 3D behavior of the BNO055
        w=0
        x=1
        y=2
        z=3
        quat=self.quat                          
        
        return np.array((np.arctan2(2.0*(quat[x]*quat[y] + quat[z]*quat[w]),(-quat[z]**2 - quat[x]**2 + quat[y]**2 + quat[w]**2)),
                         np.arctan2(2.0*(quat[z]*quat[x] + quat[y]*quat[w]),quat[z]**2 - quat[x]**2 - quat[y]**2 + quat[w]**2),
                         np.arcsin(-2.0*(quat[z]*quat[y] - quat[x]*quat[w])/(quat[z]**2 + quat[x]**2 + quat[y]**2 + quat[w]**2))),dtype='d')
    

    @property
    def imu_gyro(self):
        # convert angularvel into the object frame
        # We need to invert quat... 
        #
        # dr/dt_in_rotating frame = dr/dt_in_inertial_frame - omega x r_in_rotating_frame
        # here dr/dt in inertial frame is zero -- r will be each
        # inertial principal axis in turn.
        # therefore dr_in_rotating_frame/dt = - omega cross r_in_rotating_frame  where r = xhat, yhat or zhat
        # if r1=xhat_rotating, r2=yhat_rotating, r3=zhat_rotating
        # rotation about r1 = dy/dt x dz/dt 
        # rotation about r2 = dz/dt x dx/dt
        # rotation about r3 = dx/dt x dy/dt

        # ... as first cut let's just do the coordinate transformation
        # I think this actually covers it because angular velocity
        # is a legitimate (pseudo-)vector and thus can be
        # properly transformed like any other vector. 
        quat_inverse = quatinv(self.quat)
        
        return vec_apply_rot(quat_inverse,self.angularvel)
        
    pass

dynamic_instance=None  # This variable is assigned to an instance of dynamic_model by simulator.py on startup

#def start_if_needed():
#    global dynamic_instance
#
#    if dynamic_instance is None:
#        dynamic_instance = dynamic_model()
#        pass
#    
#    pass

