#! /usr/bin/env python
"""
Maintainer: Jahid (email: islam034@umn.edu)
Interactive Robotics and Vision Lab
http://irvlab.cs.umn.edu/

Class for generating a menu using {left, right} tokens from hand gestures
This is for menu-selection using Robo-Chat-Gest
"""


class MenuSelection:
    """ 
      Class for generating a menu using {left, right} tokens from hand gestures
       There are four menus currently in Aqua {0, 1, 2, 3, 4}
       To select a menu, perform {ok, ok} and then {menu_id, menu_id}
       e.g., to select menu #2
            > perform {left hand, right hand} = {ok, ok}
            > then    {left hand, right hand} = {1, 1} 
            > the generated instruction will be: "MENU 1"
       *** this is not in the paper, added later for menu selection
    """
  
  
    def __init__(self, catg=None):
        """ 
          Initialize variables and get ready to decode a new instruction
        """
        # categories and corresponding class tokens
        if catg is None:
          self.ys  =  ['0', '1', '2', '3', '4', '5', 'left', 'right', 'pic', 'ok', 'H']
        else:
          self.ys = catg
        self.c_num =  [0,   1,   2,   3,   4,   5,     6,      7,      8,     9,   10 ]

        # for simple gesture tokens to  menu instruction mapping
        # instruction will be like "MENU id" 
        self.STATES = { ('_', '_'): 'IDLE', ('0', '0'): '0', ('5', '5'): '5', ('1', '1'): '1', 
                        ('2', '2'): '2', ('3', '3'): '3', ('4', '4'): '4', ('ok', 'ok'): 'SELECT MENU'}
        self.CURR_State = 'IDLE'
        self.instruction_ = ''
  
  
    

    def init_FSM(self, l_token, r_token):
        # Looks for triggering state = {Go}
        if ( self.ys[l_token]=='ok' and self.ys[r_token]=='ok'):
          self.CURR_State = 'SELECT MENU'
          self.instruction_ = self.CURR_State + ' '  
  

  
    
    def update_FSM(self, l_token, r_token):
        # Once found triggering state = {Go}, if a valid menu id found, return it
        # else keep waiting for that id and return False for now
        if ((self.ys[l_token], self.ys[r_token]) in self.STATES.keys()):
          Prospective_state_ = self.STATES[(self.ys[l_token], self.ys[r_token])]
          if ((self.CURR_State == 'SELECT MENU') and (Prospective_state_ != 'SELECT MENU')):
              self.CURR_State = Prospective_state_
              self.instruction_ = self.instruction_ + self.CURR_State
              return True
        return False
 
            
  
    def decode(self, l_token, r_token):
        # simple FSM - Finite State Machine to generate the instruction
        # clear cache is the current instruction generation is done
        done_ = False
        if (self.CURR_State=='IDLE'):
          self.init_FSM(l_token, r_token)
        else:
          done_ = self.update_FSM(l_token, r_token)
          if done_:
              self.CURR_State = 'IDLE'
  
        return self.instruction_, done_
     


