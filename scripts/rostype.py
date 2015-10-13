'''
@returns(type)
def function_to_modify():
	...

This will return the value from the function if it is an instance
of the type passed into the returns. Otherwise it will
throw a TypeError

The function modified with the decorator will also have a property
return_type that gives the class that is guarunteed (neglecting Exceptions)
to be returned from the function.

Example:

from nav_msgs.msg import Odometry

@returns(Odometry)
def ogian(g):
	return g

>>> ogian.return_type
<class 'nav_msgs.msg._Odometry.Odometry'>
>>> ogian(Odometry())
returns the new Odometry instance
>>> ogian("This is a string")
throws TypeError

'''
def returns(type):
	def wrap(f):
		def wrapped_f(*args, **kwargs):
			v = f(*args, **kwargs)
			if isinstance(v, type):
				# print 'return matches type'
				return v
			else:
				# print 'return is not of type'
				raise TypeError('rostype: returns: Function did not match forced type')
		wrapped_f.return_type = type
		return wrapped_f
	return wrap