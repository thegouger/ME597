
NAME=MAPView
CC=g++
CFLAGS= -g
LIBS=-lsfml-graphics -lsfml-window -lsfml-system -lGLU

OBJDIR=./obj
SRCDIR=./src
OBJ= ${OBJDIR}/main.o  ${OBJDIR}/body.o 
HEAD=${SRCDIR}/body.hpp 

all: ${NAME}

${NAME}: ${OBJ} ${HEAD}
	${CC} ${CFLAGS} -o ${NAME} ${OBJ} ${LIBS}

${OBJDIR}/main.o: ${SRCDIR}/main.cpp ${SRCDIR}/consts.hpp
	${CC} ${CFLAGS} -o ${OBJDIR}/main.o -c ${SRCDIR}/main.cpp ${LIBS}

${OBJDIR}/body.o: ${SRCDIR}/body.cpp ${SRCDIR}/body.hpp ${SRCDIR}/consts.hpp
	${CC} ${CFLAGS} -o ${OBJDIR}/body.o -c ${SRCDIR}/body.cpp ${LIBS}


clean:
	rm -fr ${OBJDIR}/*.o
	rm -f ${NAME}

