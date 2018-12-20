#!/usr/bin/env python
"""
Maintainer: Jahid (email: islam034@umn.edu)
Interactive Robotics and Vision Lab
http://irvlab.cs.umn.edu/

Class for generating a menue using {left, right} tokens from hand gestures
"""

class InstructionGeneration:
  """ 
    Class for generating an instruction using {left, right} tokens from hand gestures
     > paper: https://ieeexplore.ieee.org/document/8543168
     *** certain instruction rules and parameters are changed for convenience, 
         for example, the 'STOP' is now {0, 0} not {ok, 0}
                      thr_frame = 2 instead of 15 (no of detections to activate a transition)
         the overall operation is the same though
  """

  def __init__(self, catg=None):
    if catg is None:
      self.ys  =  ['0', '1', '2', '3', '4', '5', 'left', 'right', 'pic', 'ok', 'H']
    else:
      self.ys = catg
    self.thr_frame = 2

    # See the state transition diagram and gesture--instruction mapping
    self.STATES = { ('_', '_'): 'IDLE', ('0', '0'): 'STOP', ('5', '5'): 'HOVER', 
                    ('5', '1'): 'FOLLOW', ('0', 'left'): 'LEFT', ('0', 'right'): 'RIGHT',
                    ('right', 'right'): 'UP', ('left', 'left'): 'DOWN', ('ok', '0'): 'Num', ('ok', '1'): 'Num',
                    ('ok', '2'): 'Num', ('ok', '3'): 'Num', ('ok', '4'): 'Num', ('ok', '5'): 'Num', 
                    ('0', 'pic'): 'Param', ('1', 'pic'): 'Param', ('2', 'pic'): 'Param', 
                    ('3', 'pic'): 'Param', ('4', 'pic'): 'Param', ('5', 'pic'): 'Param', 
                    ('pic', '0'): 'CONTD', ('pic', 'pic'): 'SNAP', ('left', 'pic'): 'Decrease', 
                    ('right', 'pic'): 'Increase', ('ok', 'pic'): 'Default', ('ok', 'ok'): 'GO',
                    ('pic', 'ok'): 'AND', ('pic', '2'): 'EXEC'}

    self.initialize_vars()

  # initializes variables before capturing a new instruction
  def initialize_vars (self):
    self.CURR_State = 'IDLE'
    self.last_seen = ['_' , '_' ]
    self.N_seen = 0
    self.instruction_ = ''


  # Looks for initial states = {STOP, CONTD}
  # Have to see a gesture for thr_frame consecutive frames to confirm correct detection
  def init_FSM(self, l_token, r_token):
    if (self.ys[l_token]=='0' and self.ys[r_token]=='0'):
      if (self.ys[l_token]==self.last_seen[0] and self.ys[r_token]==self.last_seen[1]):
        self.N_seen+=1
        if self.N_seen > self.thr_frame and self.CURR_State == 'IDLE':
          self.CURR_State = 'STOP'
          self.instruction_ = self.CURR_State + ' ' 
      else: 
        self.N_seen = 0

      
    elif (self.ys[l_token]=='pic' and self.ys[r_token]=='0'):
      if (self.ys[l_token]==self.last_seen[0] and self.ys[r_token]==self.last_seen[1]):
        self.N_seen+=1
        if self.N_seen > self.thr_frame and self.CURR_State == 'IDLE':
          self.CURR_State = 'CONTD'
          self.instruction_ = self.instruction_ + self.CURR_State + ' '
      else: 
        self.N_seen = 0

    else:
      self.N_seen=0

    self.last_seen = [self.ys[l_token], self.ys[r_token]]


  # Once found initial states, update to next state if necessary
  # Have to see a gesture for thr_frame = 15 consecutive frames to confirm correct detection
  def update_FSM(self, l_token, r_token):
    if ((self.ys[l_token], self.ys[r_token]) in self.STATES.keys()):
      if (self.ys[l_token]==self.last_seen[0] and self.ys[r_token]==self.last_seen[1]):
        self.N_seen+=1
        Prospective_state_ = self.STATES[(self.ys[l_token], self.ys[r_token])]
        if self.N_seen >= self.thr_frame and self.CURR_State != Prospective_state_:
          #print (self.CURR_State, self.last_seen)
          self.CURR_State = Prospective_state_
          if self.CURR_State == 'Num':
            self.instruction_ = self.instruction_ + self.ys[r_token] + ' '
          elif self.CURR_State == 'Param': 
            self.instruction_ = self.instruction_ + self.CURR_State + ' ' + self.ys[l_token] + ' '
          elif self.CURR_State == 'AND':
            self.instruction_ = self.instruction_.strip()  
          else:
            self.instruction_ = self.instruction_ + self.CURR_State + ' '
            print self.instruction_
      else:
        self.N_seen = 0

    self.last_seen = [self.ys[l_token], self.ys[r_token]]

    if self.CURR_State == 'GO':
      # Final GO found, instruction complete; 
      return True
    return False
          

  # start or continue FSM - Finite State Machine
  def decode(self, l_token, r_token):
    done_ = False      
    if (self.CURR_State=='IDLE'):
      self.init_FSM(l_token, r_token)
    else:
      done_ = self.update_FSM(l_token, r_token)
      if done_:
          self.CURR_State = 'IDLE'

    return self.instruction_, done_
