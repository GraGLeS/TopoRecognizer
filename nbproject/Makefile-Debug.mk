#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=Cygwin-Windows
CND_DLIB_EXT=dll
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/LinearCurve.o \
	${OBJECTDIR}/LocalPointset.o \
	${OBJECTDIR}/Plane.o \
	${OBJECTDIR}/Point3D.o \
	${OBJECTDIR}/PointCloudGenerator.o \
	${OBJECTDIR}/QuadraticCurve.o \
	${OBJECTDIR}/Spline.o \
	${OBJECTDIR}/TriplelinePointsetClass.o \
	${OBJECTDIR}/main.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/masterarbeit_fw.exe

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/masterarbeit_fw.exe: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/masterarbeit_fw ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/LinearCurve.o: LinearCurve.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/cygdrive/C/eigen-eigen-dc6cfdf9bcec -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/LinearCurve.o LinearCurve.cpp

${OBJECTDIR}/LocalPointset.o: LocalPointset.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/cygdrive/C/eigen-eigen-dc6cfdf9bcec -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/LocalPointset.o LocalPointset.cpp

${OBJECTDIR}/Plane.o: Plane.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/cygdrive/C/eigen-eigen-dc6cfdf9bcec -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Plane.o Plane.cpp

${OBJECTDIR}/Point3D.o: Point3D.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/cygdrive/C/eigen-eigen-dc6cfdf9bcec -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Point3D.o Point3D.cpp

${OBJECTDIR}/PointCloudGenerator.o: PointCloudGenerator.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/cygdrive/C/eigen-eigen-dc6cfdf9bcec -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/PointCloudGenerator.o PointCloudGenerator.cpp

${OBJECTDIR}/QuadraticCurve.o: QuadraticCurve.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/cygdrive/C/eigen-eigen-dc6cfdf9bcec -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/QuadraticCurve.o QuadraticCurve.cpp

${OBJECTDIR}/Spline.o: Spline.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/cygdrive/C/eigen-eigen-dc6cfdf9bcec -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Spline.o Spline.cpp

${OBJECTDIR}/TriplelinePointsetClass.o: TriplelinePointsetClass.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/cygdrive/C/eigen-eigen-dc6cfdf9bcec -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/TriplelinePointsetClass.o TriplelinePointsetClass.cpp

${OBJECTDIR}/main.o: main.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/cygdrive/C/eigen-eigen-dc6cfdf9bcec -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/masterarbeit_fw.exe

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
