#!/usr/bin/python

from generic_trials import *

# run_trials("cloth_table", run_baseline=True, run_UCB=True, run_KFMANB=False, run_KFMANDB=False, log_prefix="wafr_final_submission/")
# run_trials("rope_cylinder", run_baseline=True, run_UCB=True, run_KFMANB=False, run_KFMANDB=False, log_prefix="wafr_final_submission/")
run_trials("cloth_wafr", run_baseline=False, run_UCB=False, run_KFMANB=False, run_KFMANDB=True, generate_screenshots="false", log_prefix="wafr_final_submission/")
