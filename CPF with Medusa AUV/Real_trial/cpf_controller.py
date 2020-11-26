#!/usr/bin/python

import roslib 
roslib.load_manifest('event_trigger_cpf')
import rospy
import sys

from medusa_msgs.msg import mState
from std_msgs.msg import Int8
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
from event_trigger_cpf.msg import evcpf_comms

from math import atan2, pi, tan, sqrt, cos, sin, tanh
from math import exp as mexp
import numpy as np

class cpf_controller:
    def __init__(self):
        # mission
        self.x_init = 491884.65
        self.y_init = 4290810.59 

        # parameters
        self.epsilon = 0.5;
        self.tau_bar = 0.2;

        self.v_id = rospy.get_param("~vehicle_id", 0)
        self.formation = np.array([[0,0,0], [0,-5,5]]);
        # init variables pf_controller
        self.pf_p_hat_old = 0
        self.pf_Vcxy_hat_old = 0
        self.pf_gamma_dot_old = 0
        self.pf_gamma_old = 0
        self.pf_eu_old = 0
        self.pf_Com_Mode_old = 0
        self.pf_er_old = 0
        self.pf_Diff_Mode_old = 0

        # init PF variables
        self.gamma = 0;
        self.vp = 0.5;
        self.L = 30;
        self.R = 12;
        self.m = float(self.formation[0][self.v_id]);
        self.d = float(self.formation[1][self.v_id]);

        print "d, ", self.d, "m", self.m
        # init coordination variables
        self.coordinating = False;
        self.coor_e_int = 0;
        self.adjacency_matrix = np.array([[0,0,1], [1,0,0], [1,0,0]]);
        self.neighbors = self.adjacency_matrix[self.v_id]
        self.num_neighbors = np.sum(self.adjacency_matrix[self.v_id]);
        self.gamma_c_n_hat = np.zeros(self.adjacency_matrix[0].size)
        self.gamma_c0 = np.zeros(self.adjacency_matrix[0].size)
        self.gamma_cn = np.zeros(self.adjacency_matrix[0].size)
        self.time_last_rcv = [rospy.Time(0) for i in range(self.adjacency_matrix[0].size)]
        self.time_last_send = [rospy.Time(0) for i in range(self.adjacency_matrix[0].size)]

        self.t_begin = rospy.Time.now();


        self.pub_hcm_mode = rospy.Publisher('HCm_Mode', Int8, queue_size=5)
        self.pub_df_mode = rospy.Publisher('HDf_Mode', Int8, queue_size=5)
        self.pub_pferror = rospy.Publisher('pf_error', PointStamped, queue_size=5)
        self.pub_debug = rospy.Publisher('pf_debug', Float64, queue_size=5)   
        self.pub_outbox = rospy.Publisher("etcpf/outbox", evcpf_comms, queue_size = 5)

        self.sub_inbox = rospy.Subscriber("etcpf/inbox", evcpf_comms, self.receive_data, queue_size = 1)

        rospy.Subscriber("State", mState, self.positionCallback)


    def iterate(self):
        tnow = rospy.Time.now()
        # path
        [gamma_c, pd, pd_gamma, vd_bar, cur] = self.path(self.gamma,self.vp,self.L,self.R,self.d,self.m);
        p=np.matrix([self.y, self.x]).transpose();
        print "gamma_c ", gamma_c
        print "position", p
        print "pd", pd
        print "pd_gamma", pd_gamma
        print "vd_bar", vd_bar

        # coordination
        Ts = 0.1;
        vd_t = 0;
        if(self.coordinating):
            vd_t = self.coordination_controller(gamma_c, self.gamma_c_n_hat, Ts, vd_bar)

        self.gamma_c = gamma_c;
        [gamma_next,u_d,r_d, cm_mode, df_mode] = self.pf_controller(pd, pd_gamma, vd_bar, vd_t, self.u, self.r, self.yaw, p, Ts, cur);

        self.gamma=float(gamma_next[0][0]);
        #print "-------- Coordination --------"
        self.coordinating = True;
        #print "neighbors ", self.neighbors
        for neighbor_id in range(0, self.adjacency_matrix[0].size):
            if(self.neighbors[neighbor_id] == 1):
                #print "id =", neighbor_id

                if(self.time_last_rcv[neighbor_id] == rospy.Time(0)):
                    #print "not received from", neighbor_id
                    self.coordinating = False;

                self.gamma_c_n_hat[neighbor_id] = self.predict_vehicle_estimator(neighbor_id, tnow)
                self.checkEstimators(neighbor_id, tnow, gamma_c, self.vp, self.epsilon)
                #print "vd_t ", vd_t

        if(self.v_id ==0 and (tnow-self.t_begin).to_sec()>100 and (tnow-self.t_begin).to_sec()<160):
            cm_mode = 15;
        # publish cm_mode, diff_mode
        msg = Int8()
        msg.data = int(cm_mode)
        msg.data = min(max(msg.data, -40), 60);
        self.pub_hcm_mode.publish(msg)

        msg.data = int(df_mode)
        msg.data = min(max(msg.data, -60), 60);
        self.pub_df_mode.publish(msg)


    def positionCallback(self, msg):
        # if(hasattr(self, 'x_init')==False):
        #   self.x_init = msg.X;
        #   self.y_init = msg.Y;

        #print self.x_init , self.y_init 
        self.yaw = msg.Yaw*pi/180.0;
        self.r = msg.Yaw_rate;
        self.x = msg.X - self.x_init;
        self.y = msg.Y - self.y_init;
        self.u = msg.u;
        self.iterate();

    def pf_controller(self, pd, pd_gamma, vd_bar, vd_t, u, r, yaw, p, Ts, cur):
        t = (rospy.Time.now()-self.t_begin).to_sec()
        if(t<2.0):
            self.pf_p_hat_old = p;

        # Declare and initialize global variales
        p_hat_old = self.pf_p_hat_old
        Vcxy_hat_old = self.pf_Vcxy_hat_old
        gamma_dot_old = self.pf_gamma_dot_old
        gamma_old = self.pf_gamma_old
        eu_old = self.pf_eu_old
        Com_Mode_old = self.pf_Com_Mode_old
        er_old = self.pf_er_old
        Diff_Mode_old = self.pf_Diff_Mode_old

        # Calculate rotation matrix from body frame to inertial frame
        Rot=np.matrix([[cos(yaw),-sin(yaw)],[sin(yaw),cos(yaw)]]); 

        # Estimate current 
        # Estimate current in Inertial frame
        p_t=p-self.pf_p_hat_old;
        Vr=np.matrix([u,0]).transpose();
        Kp=10*np.eye(2);
        Kc=10*np.eye(2);

        p_hat_dot=Rot*Vr+self.pf_Vcxy_hat_old+Kp*p_t;
        p_hat=self.pf_p_hat_old+Ts*p_hat_dot;
        self.pf_p_hat_old=p_hat;

        Vcxy_hat_dot=Kc*p_t;
        Vcxy_hat=self.pf_Vcxy_hat_old+Ts*Vcxy_hat_dot;
        self.pf_Vcxy_hat_old=Vcxy_hat;
        # Transform the estimated current to Body frame
        Vc_hat=Rot.transpose()*Vcxy_hat;
        print "Vc_hat ", Vc_hat
        # Outerloop Controller
        vd=vd_bar+vd_t;
        ##print "p-pd", (p-pd)
        pferror = PointStamped()
        pferror.point.x = (p-pd)[0];
        pferror.point.y = (p-pd)[1];
        self.pub_pferror.publish(pferror);

        e_out=Rot.transpose()*(p-pd);


        #e_out=np.clip(e_out, -5, 5);

        print "e_out", e_out
        delta=-0.2;
        Delta_inv= 1*np.matrix([[1,0],[0,-1/delta]]);

        # if t<30:
        #   kx=0.01*t;
        #   ky=0.006*t;
        #   m=.0017*t;
        # else:
          # kx=.5;
          # ky=0.25;
          # m=0.025;
        # working for 0.5m/s
        # kx=0.3;
        # ky=0.2;

        kx=0.35;
        ky=0.35;
        m=1.0;
        # kx=0.0000000002;
        # ky=0.0000005;
        # m=1;
        # end
        cur=0;
             
        Kk=np.matrix([[kx+cur,0],[0,ky+cur]]);
        tanh_func = np.vectorize(tanh)
        ud_t=Delta_inv*(-Kk*tanh_func(e_out-np.matrix([delta,0]).transpose())-Vc_hat+Rot.transpose()*pd_gamma*vd);
        n=1;

        #print "ud_t ", ud_t
        ud=np.matrix([[n,0],[0,m+cur*0]])*ud_t;
        # desired surge and yaw rate
        u_d=min(max(ud[0],0.0),0.7);
        r_d=min(max(ud[1]*180.0/pi,-10),10);

        #print "ud ", u_d
        #print "rd ", r_d

        # Inner Loop Controller
        # r_d = np.matrix([[sin(t/10)*10]])
        eu=float(u_d-u);
        er=float(r_d-r);
        #print "er ", er

        # Speed controller
        Kp_u=20.0;
        KI_u=5.0;
        KD_u=0;
        Com_Mode=min(max(Kp_u*eu+(Ts*KI_u-Kp_u)*self.pf_eu_old+self.pf_Com_Mode_old, -60), 60);
        self.pf_Com_Mode_old=Com_Mode;
        self.pf_eu_old=eu;

        # Yaw rate controller
        Kp_r=0.5;
        Kp_r=1.2;
        KI_r=3;
        KD_r=0;
        #print "self.pf_er_old", Kp_r*(er-self.pf_er_old)

        # Diff_Mode=min(max(Kp_r*er+(Ts*KI_r-Kp_r)*self.pf_er_old+self.pf_Diff_Mode_old, -60), 60);
        # Diff_Mode=Kp_r*(er-self.pf_er_old)+Ts*KI_r*self.pf_er_old+self.pf_Diff_Mode_old;
        # Diff_Mode=Kp_r*(er-self.pf_er_old)+Ts*KI_r*self.pf_er_old+self.pf_Diff_Mode_old;
        Diff_Mode=Kp_r*er+Ts*KI_r*(er+self.pf_er_old);
        self.pf_Diff_Mode_old=Diff_Mode;
        self.pf_er_old=er;

        msg = Float64()
        msg.data = float(e_out[1]);
        self.pub_debug.publish(msg);
        # Gamma Controller
        kz=1;
        vd_dot=0; #(vd_new-vd_old)/Ts;   
        #vd_old=vd_new;

        gamma_ddot=-kz*(self.pf_gamma_dot_old-vd_bar)+vd_dot+2*(e_out.transpose()-np.matrix([delta,0]))*Rot.transpose()*pd_gamma + 1*kz*vd_t; 
        #print "gamma_ddot", gamma_ddot
        gamma_dot=self.pf_gamma_dot_old+Ts*gamma_ddot;
        gamma_next=self.pf_gamma_old+Ts*gamma_dot;
        self.pf_gamma_dot_old=gamma_dot;
        self.pf_gamma_old=gamma_next;

        return [gamma_next,u_d,r_d,Com_Mode,Diff_Mode];

    def coordination_controller(self, gamma_c, gamma_c_n_hat, Ts, vd_bar):
        # Compute correction term 
        kg=1;
        ki=2;
        decay = 0.5;

        e1=(gamma_c-(1/self.num_neighbors)*np.sum(gamma_c_n_hat));
        #e2=(gamma_c-(1/self.num_neighbors)*np.sum(gamma_c_n_hat));
        #print "COORD e1 ", e1
        #print "COORD gamma_c ", gamma_c
        #print "COORD gamma_c_n_hat ", gamma_c_n_hat

        # Accumulate error for integration term
        self.coor_e_int=(1-mexp(-decay*Ts))*self.coor_e_int+Ts*e1; 
        vd_t=-kg*e1-ki*self.coor_e_int;

        # Put saturation to avoid the vehicle return back.    
        vd_t = min(max(vd_t, -vd_bar), 0.2)

        return vd_t;

    def path(self, gamma, vp,L,R,d,m):
        s1=L-m;
        s2=s1+pi*(R-d);
        s3=s2+L;
        s4=s3+pi*(R+d);
        s5=s4+m;
        ratio1=(R-d)/R;
        ratio2=(R+d)/R;
        if gamma<s5:
            cycle=0;
        else:
            cycle=1;
        s=gamma-s5*cycle;
        c1=1.2/(R-d);
        c2=1/(R+d);
        k=.5;
        #print "s",s 
        #print"exp %.3f"%(mexp(-k*(s-s1)))
        cur=c1/(1+mexp(-k*(s-s1)))-c1/(1+mexp(-k*(s-s2)))+c2/(1+mexp(-k*(s-s3)))-c2/(1+mexp(-k*(s-s4)));
        if s<s1: # first segment of straight line;
            vd_bar=vp;
            xd=s+R+m;
            yd=0+d;
            pd_gamma=np.matrix([1,0]).transpose();      
            gamma_c=s+s5*cycle;                                  # the segment is parallel with x
        elif(s>=s1) and (s<=s2):                  # second segment, a circumference 
            vd_bar=ratio1*vp;
            psi_T=(s-s1)/(R-d);
            xd=L+R+(R-d)*sin(psi_T);   
            yd=d+(R-d)*(1-cos(psi_T));
            pd_gamma=np.matrix([cos(psi_T),sin(psi_T)]).transpose();
            gamma_c=s1+(s-s1)*R/(R-d)+s5*cycle; 
        elif (s>s2) and (s<=s3):                   # third segment, a straight line 
            vd_bar=vp;
            xd=s3-s+R;  
            yd=d+2*(R-d); 
            pd_gamma=np.matrix([-1,0]).transpose();
            gamma_c=s+d*pi+s5*cycle;
        elif (s>s3) and (s<=s4):
            vd_bar=ratio2*vp;
            xd=R-(R+d)*sin((s-s3)/(R+d));
            yd=d+2*(R-d)+(R+d)*(1-cos((s-s3)/(R+d)));
            pd_gamma=np.matrix([-cos((s-s3)/(R+d)),sin((s-s3)/(R+d))]).transpose(); 
            gamma_c=s3+d*pi+(s-s3)*R/(R+d)+s5*cycle;  
        else:
            vd_bar=vp;
            xd=(s-s4)+R;
            yd=d+4*R;
            pd_gamma=np.matrix([1,0]).transpose();
            gamma_c=s+s5*cycle;
        #print "xd ", xd, " yd ", yd
        #print "R", R
        pd=np.matrix([xd,cycle*4*R+yd]).transpose();
        # Rotate the mission
        phi=0;
        Rot=np.matrix([[cos(phi),sin(phi)],[-sin(phi),cos(phi)]]); 
        pd=Rot*pd;
        #print "pd ", pd

        pd_gamma=Rot*pd_gamma;
        return [gamma_c, pd, pd_gamma, vd_bar, cur];

    def send_data(self, id_to_send, gamma_to_send):
        msg_to_send                = evcpf_comms();
        msg_to_send.header.stamp   = rospy.Time.now();
        msg_to_send.source_id      = self.v_id;
        msg_to_send.destination_id = id_to_send;
        msg_to_send.time           = rospy.Time.now().to_sec();
        msg_to_send.gama           = gamma_to_send;
        self.pub_outbox.publish(msg_to_send);

    def receive_data(self, msg):
        if(msg.destination_id == self.v_id and self.adjacency_matrix[self.v_id][msg.source_id] == 1):
            self.update_vehicle_estimator(msg.source_id, msg.time, msg.gama);

    def update_vehicle_estimator(self, id, time, gamma):
        # rospy.logerr("UPDATING WITH ID ."+str(id))
        self.gamma_cn[id] = gamma;
        self.time_last_rcv[id] = rospy.Time(time);

    def predict_vehicle_estimator(self, id, time):
        if(self.time_last_rcv[id] == rospy.Time(0)):
            return 0;
        return self.gamma_cn[id]+self.vp*(time-self.time_last_rcv[id]).to_sec()

    def checkEstimators(self, id, time, gamma_c, vp, epsilon):
        # Run estimators of gamma
        gamma_e = gamma_c - (self.gamma_c0[id]+vp*(time-self.time_last_send[id]).to_sec())
        print "COOOORD gamma_e, ", gamma_e
        if(abs(gamma_e) > epsilon):
            self.send_data(id, gamma_c)
            self.gamma_c0[id] = gamma_c
            self.time_last_send[id] = rospy.Time.now()

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("cpf_controller")

    # Start node
    obj = cpf_controller()
    rospy.spin()
