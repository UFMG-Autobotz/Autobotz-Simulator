# preparar para rodar mundo (script deve ser rodado no mesmo processo)
. Setup/VT_setup.sh

# iniciar gazebo com mundo parado
gazebo 02_Worlds/VT_flatfield.world -u --verbose
