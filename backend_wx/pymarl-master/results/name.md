drone=3 t_env 1200
12 qmix unique_task train "results\models\qmix__2026-04-24_19-46-57"
16 qmix unique_task test

17 qmix no_unique_task train "results\models\qmix__2026-05-10_21-37-26"

drone=10 task_total=50
21 qmix no_unique_task train "backend_wx\pymarl-master\results\models\qmix__2026-05-26_20-53-15"