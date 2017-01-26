#px4_add_module(
#	MODULE examples__bent_app
#        MAIN bent_app_2
#	SRCS
#                bent_app_2.c
#                test_app.cpp
#	DEPENDS
#		platforms__common
#	)

px4_add_module(
        MODULE examples__bent_app
        MAIN   test_app
        SRCS
                test_app.cpp
        DEPENDS
                platforms__common
        )
