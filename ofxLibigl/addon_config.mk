meta:
	ADDON_NAME = ofxLibigl
	ADDON_DESCRIPTION = An openFrameworks wrapper for Libigl.
	ADDON_AUTHOR = Kye Shimizu
	ADDON_TAGS = "linear algebra" "geometry" "deep learning"
	ADDON_URL = http://github.com/kkshmz/ofxLibigl

common:
	ADDON_DEFINES =
	ADDON_CFLAGS = -O3 -Wno-strict-aliasing

	# Exclude includes and source.
	ADDON_SOURCES_EXCLUDE = libs/igl/include/%
	ADDON_INCLUDES_EXCLUDE = libs/igl/include/%


	# Manually add the includes and source.
	ADDON_INCLUDES = libs/igl/include
	ADDON_INCLUDES += libs/ofxLibigl/include
	ADDON_INCLUDES += src
