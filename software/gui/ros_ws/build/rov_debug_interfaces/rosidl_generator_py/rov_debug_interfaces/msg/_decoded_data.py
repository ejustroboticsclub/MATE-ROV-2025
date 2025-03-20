# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rov_debug_interfaces:msg/DecodedData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DecodedData(type):
    """Metaclass of message 'DecodedData'."""

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
                'rov_debug_interfaces.msg.DecodedData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__decoded_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__decoded_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__decoded_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__decoded_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__decoded_data

            from rov_debug_interfaces.msg import Imu
            if Imu.__class__._TYPE_SUPPORT is None:
                Imu.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DecodedData(metaclass=Metaclass_DecodedData):
    """Message class 'DecodedData'."""

    __slots__ = [
        '_id',
        '_valid',
        '_imu',
        '_thruster_current_1',
        '_thruster_current_2',
        '_thruster_current_3',
        '_thruster_current_4',
        '_thruster_current_5',
        '_thruster_current_6',
        '_thruster_pwm_1',
        '_thruster_pwm_2',
        '_thruster_pwm_3',
        '_thruster_pwm_4',
        '_thruster_pwm_5',
        '_thruster_pwm_6',
        '_indicator_1',
        '_indicator_2',
        '_indicator_3',
        '_indicator_4',
        '_indicator_5',
        '_indicator_6',
        '_heartbeat_1',
        '_heartbeat_2',
        '_heartbeat_3',
        '_heartbeat_4',
        '_connection_percentage_1',
        '_connection_percentage_2',
        '_connection_percentage_3',
        '_connection_percentage_4',
        '_arm_1',
        '_arm_2',
        '_arm_3',
        '_arm_4',
        '_rov_depth',
    ]

    _fields_and_field_types = {
        'id': 'int32',
        'valid': 'boolean',
        'imu': 'rov_debug_interfaces/Imu',
        'thruster_current_1': 'float',
        'thruster_current_2': 'float',
        'thruster_current_3': 'float',
        'thruster_current_4': 'float',
        'thruster_current_5': 'float',
        'thruster_current_6': 'float',
        'thruster_pwm_1': 'float',
        'thruster_pwm_2': 'float',
        'thruster_pwm_3': 'float',
        'thruster_pwm_4': 'float',
        'thruster_pwm_5': 'float',
        'thruster_pwm_6': 'float',
        'indicator_1': 'float',
        'indicator_2': 'float',
        'indicator_3': 'float',
        'indicator_4': 'float',
        'indicator_5': 'float',
        'indicator_6': 'float',
        'heartbeat_1': 'boolean',
        'heartbeat_2': 'boolean',
        'heartbeat_3': 'boolean',
        'heartbeat_4': 'boolean',
        'connection_percentage_1': 'float',
        'connection_percentage_2': 'float',
        'connection_percentage_3': 'float',
        'connection_percentage_4': 'float',
        'arm_1': 'float',
        'arm_2': 'float',
        'arm_3': 'float',
        'arm_4': 'float',
        'rov_depth': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['rov_debug_interfaces', 'msg'], 'Imu'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        self.id = kwargs.get('id', int())
        self.valid = kwargs.get('valid', bool())
        from rov_debug_interfaces.msg import Imu
        self.imu = kwargs.get('imu', Imu())
        self.thruster_current_1 = kwargs.get('thruster_current_1', float())
        self.thruster_current_2 = kwargs.get('thruster_current_2', float())
        self.thruster_current_3 = kwargs.get('thruster_current_3', float())
        self.thruster_current_4 = kwargs.get('thruster_current_4', float())
        self.thruster_current_5 = kwargs.get('thruster_current_5', float())
        self.thruster_current_6 = kwargs.get('thruster_current_6', float())
        self.thruster_pwm_1 = kwargs.get('thruster_pwm_1', float())
        self.thruster_pwm_2 = kwargs.get('thruster_pwm_2', float())
        self.thruster_pwm_3 = kwargs.get('thruster_pwm_3', float())
        self.thruster_pwm_4 = kwargs.get('thruster_pwm_4', float())
        self.thruster_pwm_5 = kwargs.get('thruster_pwm_5', float())
        self.thruster_pwm_6 = kwargs.get('thruster_pwm_6', float())
        self.indicator_1 = kwargs.get('indicator_1', float())
        self.indicator_2 = kwargs.get('indicator_2', float())
        self.indicator_3 = kwargs.get('indicator_3', float())
        self.indicator_4 = kwargs.get('indicator_4', float())
        self.indicator_5 = kwargs.get('indicator_5', float())
        self.indicator_6 = kwargs.get('indicator_6', float())
        self.heartbeat_1 = kwargs.get('heartbeat_1', bool())
        self.heartbeat_2 = kwargs.get('heartbeat_2', bool())
        self.heartbeat_3 = kwargs.get('heartbeat_3', bool())
        self.heartbeat_4 = kwargs.get('heartbeat_4', bool())
        self.connection_percentage_1 = kwargs.get('connection_percentage_1', float())
        self.connection_percentage_2 = kwargs.get('connection_percentage_2', float())
        self.connection_percentage_3 = kwargs.get('connection_percentage_3', float())
        self.connection_percentage_4 = kwargs.get('connection_percentage_4', float())
        self.arm_1 = kwargs.get('arm_1', float())
        self.arm_2 = kwargs.get('arm_2', float())
        self.arm_3 = kwargs.get('arm_3', float())
        self.arm_4 = kwargs.get('arm_4', float())
        self.rov_depth = kwargs.get('rov_depth', float())

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
        if self.id != other.id:
            return False
        if self.valid != other.valid:
            return False
        if self.imu != other.imu:
            return False
        if self.thruster_current_1 != other.thruster_current_1:
            return False
        if self.thruster_current_2 != other.thruster_current_2:
            return False
        if self.thruster_current_3 != other.thruster_current_3:
            return False
        if self.thruster_current_4 != other.thruster_current_4:
            return False
        if self.thruster_current_5 != other.thruster_current_5:
            return False
        if self.thruster_current_6 != other.thruster_current_6:
            return False
        if self.thruster_pwm_1 != other.thruster_pwm_1:
            return False
        if self.thruster_pwm_2 != other.thruster_pwm_2:
            return False
        if self.thruster_pwm_3 != other.thruster_pwm_3:
            return False
        if self.thruster_pwm_4 != other.thruster_pwm_4:
            return False
        if self.thruster_pwm_5 != other.thruster_pwm_5:
            return False
        if self.thruster_pwm_6 != other.thruster_pwm_6:
            return False
        if self.indicator_1 != other.indicator_1:
            return False
        if self.indicator_2 != other.indicator_2:
            return False
        if self.indicator_3 != other.indicator_3:
            return False
        if self.indicator_4 != other.indicator_4:
            return False
        if self.indicator_5 != other.indicator_5:
            return False
        if self.indicator_6 != other.indicator_6:
            return False
        if self.heartbeat_1 != other.heartbeat_1:
            return False
        if self.heartbeat_2 != other.heartbeat_2:
            return False
        if self.heartbeat_3 != other.heartbeat_3:
            return False
        if self.heartbeat_4 != other.heartbeat_4:
            return False
        if self.connection_percentage_1 != other.connection_percentage_1:
            return False
        if self.connection_percentage_2 != other.connection_percentage_2:
            return False
        if self.connection_percentage_3 != other.connection_percentage_3:
            return False
        if self.connection_percentage_4 != other.connection_percentage_4:
            return False
        if self.arm_1 != other.arm_1:
            return False
        if self.arm_2 != other.arm_2:
            return False
        if self.arm_3 != other.arm_3:
            return False
        if self.arm_4 != other.arm_4:
            return False
        if self.rov_depth != other.rov_depth:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'id' field must be an integer in [-2147483648, 2147483647]"
        self._id = value

    @builtins.property
    def valid(self):
        """Message field 'valid'."""
        return self._valid

    @valid.setter
    def valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'valid' field must be of type 'bool'"
        self._valid = value

    @builtins.property
    def imu(self):
        """Message field 'imu'."""
        return self._imu

    @imu.setter
    def imu(self, value):
        if __debug__:
            from rov_debug_interfaces.msg import Imu
            assert \
                isinstance(value, Imu), \
                "The 'imu' field must be a sub message of type 'Imu'"
        self._imu = value

    @builtins.property
    def thruster_current_1(self):
        """Message field 'thruster_current_1'."""
        return self._thruster_current_1

    @thruster_current_1.setter
    def thruster_current_1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thruster_current_1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'thruster_current_1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._thruster_current_1 = value

    @builtins.property
    def thruster_current_2(self):
        """Message field 'thruster_current_2'."""
        return self._thruster_current_2

    @thruster_current_2.setter
    def thruster_current_2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thruster_current_2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'thruster_current_2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._thruster_current_2 = value

    @builtins.property
    def thruster_current_3(self):
        """Message field 'thruster_current_3'."""
        return self._thruster_current_3

    @thruster_current_3.setter
    def thruster_current_3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thruster_current_3' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'thruster_current_3' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._thruster_current_3 = value

    @builtins.property
    def thruster_current_4(self):
        """Message field 'thruster_current_4'."""
        return self._thruster_current_4

    @thruster_current_4.setter
    def thruster_current_4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thruster_current_4' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'thruster_current_4' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._thruster_current_4 = value

    @builtins.property
    def thruster_current_5(self):
        """Message field 'thruster_current_5'."""
        return self._thruster_current_5

    @thruster_current_5.setter
    def thruster_current_5(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thruster_current_5' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'thruster_current_5' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._thruster_current_5 = value

    @builtins.property
    def thruster_current_6(self):
        """Message field 'thruster_current_6'."""
        return self._thruster_current_6

    @thruster_current_6.setter
    def thruster_current_6(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thruster_current_6' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'thruster_current_6' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._thruster_current_6 = value

    @builtins.property
    def thruster_pwm_1(self):
        """Message field 'thruster_pwm_1'."""
        return self._thruster_pwm_1

    @thruster_pwm_1.setter
    def thruster_pwm_1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thruster_pwm_1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'thruster_pwm_1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._thruster_pwm_1 = value

    @builtins.property
    def thruster_pwm_2(self):
        """Message field 'thruster_pwm_2'."""
        return self._thruster_pwm_2

    @thruster_pwm_2.setter
    def thruster_pwm_2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thruster_pwm_2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'thruster_pwm_2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._thruster_pwm_2 = value

    @builtins.property
    def thruster_pwm_3(self):
        """Message field 'thruster_pwm_3'."""
        return self._thruster_pwm_3

    @thruster_pwm_3.setter
    def thruster_pwm_3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thruster_pwm_3' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'thruster_pwm_3' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._thruster_pwm_3 = value

    @builtins.property
    def thruster_pwm_4(self):
        """Message field 'thruster_pwm_4'."""
        return self._thruster_pwm_4

    @thruster_pwm_4.setter
    def thruster_pwm_4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thruster_pwm_4' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'thruster_pwm_4' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._thruster_pwm_4 = value

    @builtins.property
    def thruster_pwm_5(self):
        """Message field 'thruster_pwm_5'."""
        return self._thruster_pwm_5

    @thruster_pwm_5.setter
    def thruster_pwm_5(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thruster_pwm_5' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'thruster_pwm_5' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._thruster_pwm_5 = value

    @builtins.property
    def thruster_pwm_6(self):
        """Message field 'thruster_pwm_6'."""
        return self._thruster_pwm_6

    @thruster_pwm_6.setter
    def thruster_pwm_6(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thruster_pwm_6' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'thruster_pwm_6' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._thruster_pwm_6 = value

    @builtins.property
    def indicator_1(self):
        """Message field 'indicator_1'."""
        return self._indicator_1

    @indicator_1.setter
    def indicator_1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'indicator_1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'indicator_1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._indicator_1 = value

    @builtins.property
    def indicator_2(self):
        """Message field 'indicator_2'."""
        return self._indicator_2

    @indicator_2.setter
    def indicator_2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'indicator_2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'indicator_2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._indicator_2 = value

    @builtins.property
    def indicator_3(self):
        """Message field 'indicator_3'."""
        return self._indicator_3

    @indicator_3.setter
    def indicator_3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'indicator_3' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'indicator_3' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._indicator_3 = value

    @builtins.property
    def indicator_4(self):
        """Message field 'indicator_4'."""
        return self._indicator_4

    @indicator_4.setter
    def indicator_4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'indicator_4' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'indicator_4' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._indicator_4 = value

    @builtins.property
    def indicator_5(self):
        """Message field 'indicator_5'."""
        return self._indicator_5

    @indicator_5.setter
    def indicator_5(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'indicator_5' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'indicator_5' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._indicator_5 = value

    @builtins.property
    def indicator_6(self):
        """Message field 'indicator_6'."""
        return self._indicator_6

    @indicator_6.setter
    def indicator_6(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'indicator_6' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'indicator_6' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._indicator_6 = value

    @builtins.property
    def heartbeat_1(self):
        """Message field 'heartbeat_1'."""
        return self._heartbeat_1

    @heartbeat_1.setter
    def heartbeat_1(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'heartbeat_1' field must be of type 'bool'"
        self._heartbeat_1 = value

    @builtins.property
    def heartbeat_2(self):
        """Message field 'heartbeat_2'."""
        return self._heartbeat_2

    @heartbeat_2.setter
    def heartbeat_2(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'heartbeat_2' field must be of type 'bool'"
        self._heartbeat_2 = value

    @builtins.property
    def heartbeat_3(self):
        """Message field 'heartbeat_3'."""
        return self._heartbeat_3

    @heartbeat_3.setter
    def heartbeat_3(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'heartbeat_3' field must be of type 'bool'"
        self._heartbeat_3 = value

    @builtins.property
    def heartbeat_4(self):
        """Message field 'heartbeat_4'."""
        return self._heartbeat_4

    @heartbeat_4.setter
    def heartbeat_4(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'heartbeat_4' field must be of type 'bool'"
        self._heartbeat_4 = value

    @builtins.property
    def connection_percentage_1(self):
        """Message field 'connection_percentage_1'."""
        return self._connection_percentage_1

    @connection_percentage_1.setter
    def connection_percentage_1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'connection_percentage_1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'connection_percentage_1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._connection_percentage_1 = value

    @builtins.property
    def connection_percentage_2(self):
        """Message field 'connection_percentage_2'."""
        return self._connection_percentage_2

    @connection_percentage_2.setter
    def connection_percentage_2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'connection_percentage_2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'connection_percentage_2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._connection_percentage_2 = value

    @builtins.property
    def connection_percentage_3(self):
        """Message field 'connection_percentage_3'."""
        return self._connection_percentage_3

    @connection_percentage_3.setter
    def connection_percentage_3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'connection_percentage_3' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'connection_percentage_3' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._connection_percentage_3 = value

    @builtins.property
    def connection_percentage_4(self):
        """Message field 'connection_percentage_4'."""
        return self._connection_percentage_4

    @connection_percentage_4.setter
    def connection_percentage_4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'connection_percentage_4' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'connection_percentage_4' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._connection_percentage_4 = value

    @builtins.property
    def arm_1(self):
        """Message field 'arm_1'."""
        return self._arm_1

    @arm_1.setter
    def arm_1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'arm_1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'arm_1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._arm_1 = value

    @builtins.property
    def arm_2(self):
        """Message field 'arm_2'."""
        return self._arm_2

    @arm_2.setter
    def arm_2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'arm_2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'arm_2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._arm_2 = value

    @builtins.property
    def arm_3(self):
        """Message field 'arm_3'."""
        return self._arm_3

    @arm_3.setter
    def arm_3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'arm_3' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'arm_3' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._arm_3 = value

    @builtins.property
    def arm_4(self):
        """Message field 'arm_4'."""
        return self._arm_4

    @arm_4.setter
    def arm_4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'arm_4' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'arm_4' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._arm_4 = value

    @builtins.property
    def rov_depth(self):
        """Message field 'rov_depth'."""
        return self._rov_depth

    @rov_depth.setter
    def rov_depth(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rov_depth' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'rov_depth' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._rov_depth = value
