# generated from rosidl_generator_py/resource/_idl.py.em
# with input from par_interfaces:msg/GripperInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GripperInfo(type):
    """Metaclass of message 'GripperInfo'."""

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
            module = import_type_support('par_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'par_interfaces.msg.GripperInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gripper_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gripper_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gripper_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gripper_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gripper_info

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GripperInfo(metaclass=Metaclass_GripperInfo):
    """Message class 'GripperInfo'."""

    __slots__ = [
        '_gripper_type',
        '_ip',
        '_port',
        '_max_width',
        '_max_force',
    ]

    _fields_and_field_types = {
        'gripper_type': 'string',
        'ip': 'string',
        'port': 'string',
        'max_width': 'int32',
        'max_force': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.gripper_type = kwargs.get('gripper_type', str())
        self.ip = kwargs.get('ip', str())
        self.port = kwargs.get('port', str())
        self.max_width = kwargs.get('max_width', int())
        self.max_force = kwargs.get('max_force', int())

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
        if self.gripper_type != other.gripper_type:
            return False
        if self.ip != other.ip:
            return False
        if self.port != other.port:
            return False
        if self.max_width != other.max_width:
            return False
        if self.max_force != other.max_force:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def gripper_type(self):
        """Message field 'gripper_type'."""
        return self._gripper_type

    @gripper_type.setter
    def gripper_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'gripper_type' field must be of type 'str'"
        self._gripper_type = value

    @builtins.property
    def ip(self):
        """Message field 'ip'."""
        return self._ip

    @ip.setter
    def ip(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'ip' field must be of type 'str'"
        self._ip = value

    @builtins.property
    def port(self):
        """Message field 'port'."""
        return self._port

    @port.setter
    def port(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'port' field must be of type 'str'"
        self._port = value

    @builtins.property
    def max_width(self):
        """Message field 'max_width'."""
        return self._max_width

    @max_width.setter
    def max_width(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'max_width' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'max_width' field must be an integer in [-2147483648, 2147483647]"
        self._max_width = value

    @builtins.property
    def max_force(self):
        """Message field 'max_force'."""
        return self._max_force

    @max_force.setter
    def max_force(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'max_force' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'max_force' field must be an integer in [-2147483648, 2147483647]"
        self._max_force = value
