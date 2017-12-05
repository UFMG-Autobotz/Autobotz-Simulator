# preparar para rodar mundo (script deve ser rodado no mesmo processo)
. Setup/vss.sh

# iniciar gazebo com mundo parado
gazebo Worlds/VSS_penalty.world -u --verbose
