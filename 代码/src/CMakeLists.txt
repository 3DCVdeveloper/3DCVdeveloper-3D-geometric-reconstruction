set(SOURCE 
orbbec.cpp timestamp.cpp
)

add_library(myorbbec SHARED ${SOURCE})
target_link_libraries(myorbbec ${THIRD_PARTY_LIBS} 
)