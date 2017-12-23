# preparar para rodar mundo (script deve ser rodado no mesmo processo)
. Setup/VSS_setup.sh

# iniciar gazebo com mundo parado
gzserver Worlds/TEST_debug_link.world -u --verbose
