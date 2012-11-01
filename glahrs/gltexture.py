# gltexture.py

"""OpenGL texture object abstraction.

Provides a class that is an abstraction of OpenGL texture objects. It
can create textures from image files, and automatically generates
mipmaps if requested. Requires PyOpenGL and Python Imaging Library.

"""

from PIL import Image
from OpenGL.GL import *
from OpenGL.GLU import *



def _texture_image_2d(image, mipmap):
	if image.mode == 'RGB':
		data = image.tostring("raw","RGBX",0,-1)
	elif image.mode == 'RGBA':
		data = image.tostring("raw","RGBA",0,-1)
	else:
		assert False, image.mode
	width, height = image.size
	if mipmap:
		gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA, width, height,
				  GL_RGBA, GL_UNSIGNED_BYTE, data)
	else:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height,
			     0, GL_RGBA, GL_UNSIGNED_BYTE, data)


def _texture_image_1d(image, mipmap):
	if image.mode == 'RGB':
		data = image.tostring("raw","RGBX",0,-1)
	elif image.mode == 'RGBA':
		data = image.tostring("raw","RGBA",0,-1)
	else:
		assert False, image.mode
	length = max(image.size)
	if mipmap:
		gluBuild1DMipmaps(GL_TEXTURE_1D, GL_RGBA, length,
				  GL_RGBA, GL_UNSIGNED_BYTE, data)
	else:
		glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA, length,
			     0, GL_RGBA, GL_UNSIGNED_BYTE, data)


def _up_power_of_2(n):
	assert n > 0
	i = 1
	while i < n:
		i *= 2
	return i


def _scale_to_power_of_2(image):
	width, height = image.size
	pw = _up_power_of_2(width)
	ph = _up_power_of_2(height)
	if (width,height) != (pw,ph):
		return image.resize((pw,ph),Image.BICUBIC)
	return image


def _create_texture_from_image(image, wrap_s, wrap_t, magfilter, minfilter):
	if image.mode not in('RGB', 'RGBA'):
		image = image.convert("RGB")

	width,height = image.size
	#image = _scale_to_power_of_2(image)

	mipmap_filters = (GL_NEAREST_MIPMAP_NEAREST,
			  GL_NEAREST_MIPMAP_LINEAR,
			  GL_LINEAR_MIPMAP_NEAREST,
			  GL_LINEAR_MIPMAP_LINEAR)
	mipmap = minfilter in mipmap_filters

	ti = glGenTextures(1)

	if width == 1 or height == 1:
		glBindTexture(GL_TEXTURE_1D,ti)
		glTexParameterf(GL_TEXTURE_1D,GL_TEXTURE_WRAP_S,wrap_s)
		glTexParameterf(GL_TEXTURE_1D,GL_TEXTURE_MAG_FILTER,magfilter)
		glTexParameterf(GL_TEXTURE_1D,GL_TEXTURE_MIN_FILTER,minfilter)

		_texture_image_1d(image,mipmap)

		dim = GL_TEXTURE_1D

	else:
		glBindTexture(GL_TEXTURE_2D,ti)
		glPixelStorei(GL_UNPACK_ALIGNMENT,1)
		glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,wrap_s)
		glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,wrap_t)
		glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,magfilter)
		glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,minfilter)

		_texture_image_2d(image,mipmap)

		dim = GL_TEXTURE_2D

	return ti, dim, (width,height)


class Texture(object):
	"""An OpenGL texture object.

	This class uses PIL to create and bind an OpenGL texture
	object.

	Features:

	    Does 1-D and 2-D textures automatically.

	    It automatically creates mipmaps if a mipmap rendering
	    option is selected.

	    Handles alpha channel in images.

	Methods:

	     tex.enable() - enable OpenGL texturing of proper dimensions
	     tex.disabe() - disable OpenGL texturing of proper dimensions
	     tex.bind() - bind this texture
	     tex.real() - whether this is really a texture (which it is)
	     tex.destroy() - delete OpenGL texture object

	"""

	def __init__(self, flo, wrap_s=GL_REPEAT, wrap_t=GL_REPEAT,
		     magfilter=GL_LINEAR, minfilter=GL_LINEAR):
		"""Create a GL texture object.

		    tex = Texture(flo, wrap_s=GL_REPEAT, wrap_s=GL_REPEAT,
		            magfilter=GL_LINEAR, minfilter=GL_LINEAR)

		flo is a filename or a file-like object from which the
		image data comes.

		wrap_s, wrap_t, magfilter, and minfilter are some
		OpenGL texture options.

		"""

		image = Image.open(flo)
		tmp = _create_texture_from_image(
			image,wrap_s,wrap_t,magfilter,minfilter)
		self.index, self.dim, self.size = tmp

	def real(self):
		"""Return True if this is really a texture"""
		return True

	def enable(self):
		"""Enable OpenGL texturing of the proper dimensions."""
		glEnable(self.dim)

	def disable(self):
		"""Disable OpenGL texturing of the proper dimensions."""
		glDisable(self.dim)

	def bind(self):
		"""Bind this texture."""
		glBindTexture(self.dim, self.index)

	def destroy(self):
		"""Destroy this texture; release OpenGL texture object."""
		glDeleteTexture((self.index,))
		del self.index
