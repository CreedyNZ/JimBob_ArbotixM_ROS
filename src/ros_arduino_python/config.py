# Define Python dictionary of key values

coords = {
    'i_Offset': 0,
    'i_TarPos': [0, 0],
    'i_CurPos': [0, 0]}

atrib = {'i_ComMode': 0,
         'i_leftV': 0,
         'i_leftH': 0,
         'i_RightV': 0,
         'i_RightH': 0,
         'i_Buttons': 0,
         'i_Ext': 0,
         'i_Bal': 0,
         'i_Mode': 0,
         'i_Gait': 0}

# attributes for a standard packet
stdatrib = ['i_ComMode',
            'i_leftV',
            'i_leftH',
            'i_RightV',
            'i_RightH',
            'i_Buttons',
            'i_Ext']

# attributes with negative values
addatrib = ['i_leftV',
            'i_leftH',
            'i_RightV',
            'i_RightH']
