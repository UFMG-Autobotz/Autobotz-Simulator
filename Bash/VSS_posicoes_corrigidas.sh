# preparar para rodar mundo (script deve ser rodado no mesmo processo)
. Setup/VSS_setup.sh

# iniciar gazebo com mundo parado
gazebo Worlds/VSS_posicoes_corrigidas.world -u --verbose
