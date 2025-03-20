# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rov_debug_interfaces:msg/Imu.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Imu(type):
    """Metaclass of message 'Imu'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rov_debug_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rov_debug_interfaces.msg.Imu')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__imu
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__imu
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__imu
            cls._TYPE_SUPPORT = module.type_support_msg__msg__imu
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__imu

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Imu(metaclass=Metaclass_Imu):
    """Message class 'Imu'."""

    __slots__ = [
        '_acc_x',
        '_acc_y',
        '_acc_z',
        '_imu_roll',
        '_imu_pitch',
        '_imu_yaw',
    ]

    _fields_and_field_types = {
        'acc_x': 'float',
        'acc_y': 'float',
        'acc_z': 'float',
        'imu_roll': 'float',
        'imu_pitch': 'float',
        'imu_yaw': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.acc_x = kwargs.get('acc_x', float())
        self.acc_y = kwargs.get('acc_y', float())
        self.acc_z = kwargs.get('acc_z', float())
        self.imu_roll = kwargs.get('imu_roll', float())
        self.imu_pitch = kwargs.get('imu_pitch', float())
        self.imu_yaw = kwargs.get('imu_yaw', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.acc_x != other.acc_x:
            return False
        if self.acc_y != other.acc_y:
            return False
        if self.acc_z != other.acc_z:
            return False
        if self.imu_roll != other.imu_roll:
            return False
        if self.imu_pitch != other.imu_pitch:
            return False
        if self.imu_yaw != other.imu_yaw:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def acc_x(self):
        """Message field 'acc_x'."""
        return self._acc_x

    @acc_x.setter
    def acc_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'acc_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'acc_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._acc_x = value

    @builtins.property
    def acc_y(self):
        """Message field 'acc_y'."""
        return self._acc_y

    @acc_y.setter
    def acc_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'acc_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'acc_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._acc_y = value

    @builtins.property
    def acc_z(self):
        """Message field 'acc_z'."""
        return self._acc_z

    @acc_z.setter
    def acc_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'acc_z' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'acc_z' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._acc_z = value

    @builtins.property
    def imu_roll(self):
        """Message field 'imu_roll'."""
        return self._imu_roll

    @imu_roll.setter
    def imu_roll(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_roll' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'imu_roll' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._imu_roll = value

    @builtins.property
    def imu_pitch(self):
        """Message field 'imu_pitch'."""
        return self._imu_pitch

    @imu_pitch.setter
    def imu_pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_pitch' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'imu_pitch' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._imu_pitch = value

    @builtins.property
    def imu_yaw(self):
        """Message field 'imu_yaw'."""
        return self._imu_yaw

    @imu_yaw.setter
    def imu_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_yaw' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'imu_yaw' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._imu_yaw = value
