meta:
	ADDON_NAME = ofxEigen
	ADDON_DESCRIPTION = An openFrameworks wrapper for Eigen.
	ADDON_AUTHOR = Christopher Baker
	ADDON_TAGS = "linear algebra" "machine learning" "deep learning"
	ADDON_URL = http://github.com/bakercp/ofxEigen

common:
	ADDON_DEFINES =
	ADDON_CFLAGS = -O3 -Wno-strict-aliasing

	# Exclude includes and source.
	ADDON_SOURCES_EXCLUDE = libs/eigen3/include/%
	ADDON_INCLUDES_EXCLUDE = libs/eigen3/include/%

	ADDON_SOURCES_EXCLUDE += libs/eigen3-nnls/include/%
	ADDON_INCLUDES_EXCLUDE += libs/eigen3-nnls/include/%


	# Manually add the includes and source.
	ADDON_INCLUDES = libs/eigen3/include
	ADDON_INCLUDES += libs/eigen3-nnls/include
	ADDON_INCLUDES += libs/ofxEigen/include
	ADDON_INCLUDES += src
