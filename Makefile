#
#  Makefile
#  RobotLocalization
#

# Determine the platform
UNAME_S := $(shell uname -s)

# CC
ifeq ($(UNAME_S),Darwin)
  CC := clang++ -arch x86_64
else
  CC := g++
endif

# Folders
SRCDIR := src
BUILDDIR := build
TARGETDIR := bin

# Targets
EXECUTABLE := robot_localization
TARGET := $(TARGETDIR)/$(EXECUTABLE)

# Final Paths
INSTALLBINDIR := /usr/local/bin

# Code Lists
SRCEXT := cc
SOURCES := $(shell find $(SRCDIR) -type f -name *.$(SRCEXT))
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.o))

# Folder Lists
# Note: Intentionally excludes the root of the include folder so the lists are clean
INCEXT := hh
INCDIRS := $(shell find include/* -name *.$(INCEXT) -exec dirname {} \; | sort | uniq)
INCLIST := $(patsubst include/%,-I include/%,$(INCDIRS))
BUILDLIST := $(patsubst include/%,$(BUILDDIR)/%,$(INCDIRS))

# Shared Compiler Flags
CFLAGS := -c
OPENCV_INC := /usr/local/include/opencv4
OPENCV_LIB := -lopencv_gapi -lopencv_stitching -lopencv_aruco -lopencv_bgsegm \
 -lopencv_bioinspired -lopencv_ccalib -lopencv_dnn_objdetect -lopencv_dpm \
 -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hfs -lopencv_img_hash \
 -lopencv_line_descriptor -lopencv_quality -lopencv_reg -lopencv_rgbd \
 -lopencv_saliency -lopencv_sfm -lopencv_stereo -lopencv_structured_light \
 -lopencv_phase_unwrapping -lopencv_superres -lopencv_optflow \
 -lopencv_surface_matching -lopencv_tracking -lopencv_datasets \
 -lopencv_text -lopencv_highgui -lopencv_dnn -lopencv_plot -lopencv_videostab \
 -lopencv_video -lopencv_videoio -lopencv_xfeatures2d -lopencv_shape \
 -lopencv_ml -lopencv_ximgproc -lopencv_xobjdetect -lopencv_objdetect \
 -lopencv_calib3d -lopencv_imgcodecs -lopencv_features2d -lopencv_flann \
 -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core
INC := -I $(INCLIST) -I /usr/local/include -I $(OPENCV_INC)
LIB := -L /usr/local/lib $(OPENCV_LIB)

# Platform Specific Compiler Flags
ifeq ($(UNAME_S),Linux)
    CFLAGS += -std=gnu++11 -O2 # -fPIC
else
  CFLAGS += -std=c++11 -stdlib=libc++ -O2
endif

$(TARGET): $(OBJECTS)
	@mkdir -p $(TARGETDIR)
	@echo "Linking..."
	@echo "  Linking $(TARGET)"; $(CC) $^ -o $(TARGET) $(LIB)

$(BUILDDIR)/%.o: $(SRCDIR)/%.$(SRCEXT)
	@mkdir -p $(BUILDDIR)
	@echo "Compiling $<..."; $(CC) $(CFLAGS) $(INC) -c -o $@ $<

clean:
	@echo "Cleaning $(TARGET)..."; $(RM) -r $(BUILDDIR)/* $(TARGET)

.PHONY: clean
