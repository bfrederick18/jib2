GRAVITY = 9.81
DRAG_COEFFICIENT = 0.05

X_PELVIS = 0.0
Y_PELVIS = 0.5
Z_PELVIS_TOP = 0.85
Z_PELVIS_LOW = 0.85
Z_PELVIS_MID = (Z_PELVIS_TOP + Z_PELVIS_LOW) / 2
Z_PELVIS_AMP = (Z_PELVIS_TOP - Z_PELVIS_LOW) / 2
Z_PELVIS_PER_PI = 1 / 4

THROW_X_RHAND = 0.5
THROW_Y_RHAND_BACK = -0.8
THROW_Y_RHAND_FRONT = 0.9
THROW_Y_RHAND_MID = (THROW_Y_RHAND_BACK + THROW_Y_RHAND_FRONT) / 2
THROW_Y_RHAND_AMP = (THROW_Y_RHAND_FRONT - THROW_Y_RHAND_BACK) / 2
THROW_Y_RHAND_PER_PI = 1
THROW_Y_RHAND_SHIFT_PI = 1
THROW_Z_RHAND = 0.8


JOINT_NAMES = ['l_leg_hpx', 'l_leg_hpy', 'l_leg_hpz',
              'l_leg_kny',
              'l_leg_akx', 'l_leg_aky',

              'r_leg_hpx', 'r_leg_hpy', 'r_leg_hpz',
              'r_leg_kny',
              'r_leg_akx', 'r_leg_aky',

              'back_bkx', 'back_bky', 'back_bkz',
              'neck_ry',

              'l_arm_elx', 'l_arm_ely',
              'l_arm_shx', 'l_arm_shz',
              'l_arm_wrx', 'l_arm_wry', 'l_arm_wry2',

              'r_arm_elx', 'r_arm_ely',
              'r_arm_shx', 'r_arm_shz',
              'r_arm_wrx', 'r_arm_wry', 'r_arm_wry2']


JOINT_ORDERS = {
    'l_leg': [
        JOINT_NAMES.index('l_leg_hpz'), 
        JOINT_NAMES.index('l_leg_hpx'), 
        JOINT_NAMES.index('l_leg_hpy'), 
        JOINT_NAMES.index('l_leg_kny'), 
        JOINT_NAMES.index('l_leg_aky'), 
        JOINT_NAMES.index('l_leg_akx')
    ], 

    'r_leg': [
        JOINT_NAMES.index('r_leg_hpz'), 
        JOINT_NAMES.index('r_leg_hpx'), 
        JOINT_NAMES.index('r_leg_hpy'), 
        JOINT_NAMES.index('r_leg_kny'), 
        JOINT_NAMES.index('r_leg_aky'), 
        JOINT_NAMES.index('r_leg_akx')
    ], 

    'head': [
        JOINT_NAMES.index('back_bkz'), 
        JOINT_NAMES.index('back_bky'), 
        JOINT_NAMES.index('back_bkx'), 
        JOINT_NAMES.index('neck_ry')
    ], 

    'l_arm': [
        JOINT_NAMES.index('back_bkz'), 
        JOINT_NAMES.index('back_bky'), 
        JOINT_NAMES.index('back_bkx'), 
        JOINT_NAMES.index('l_arm_shz'), 
        JOINT_NAMES.index('l_arm_shx'), 
        JOINT_NAMES.index('l_arm_ely'), 
        JOINT_NAMES.index('l_arm_elx'), 
        JOINT_NAMES.index('l_arm_wry'), 
        JOINT_NAMES.index('l_arm_wrx'), 
        JOINT_NAMES.index('l_arm_wry2')],

    'r_arm': [
        JOINT_NAMES.index('back_bkz'), 
        JOINT_NAMES.index('back_bky'), 
        JOINT_NAMES.index('back_bkx'), 
        JOINT_NAMES.index('r_arm_shz'), 
        JOINT_NAMES.index('r_arm_shx'), 
        JOINT_NAMES.index('r_arm_ely'), 
        JOINT_NAMES.index('r_arm_elx'), 
        JOINT_NAMES.index('r_arm_wry'), 
        JOINT_NAMES.index('r_arm_wrx'), 
        JOINT_NAMES.index('r_arm_wry2')]
}


JOINT_INDEXES = {
    'l_leg': [JOINT_NAMES[i] for i in JOINT_ORDERS['l_leg']],
    'r_leg': [JOINT_NAMES[i] for i in JOINT_ORDERS['r_leg']],
    'head': [JOINT_NAMES[i] for i in JOINT_ORDERS['head']],
    'l_arm': [JOINT_NAMES[i] for i in JOINT_ORDERS['l_arm']],
    'r_arm': [JOINT_NAMES[i] for i in JOINT_ORDERS['r_arm']]
}


def joint_builder(array_from, joint_order):
    return [array_from[i] for i in JOINT_ORDERS[joint_order]]