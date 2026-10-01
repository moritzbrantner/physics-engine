use physics_engine::StepReport;

pub fn semantic_report(mut report: StepReport) -> StepReport {
    // Normalize only disposable capacity and cache work; physical/event and
    // equivalent query work remain compared. Cache counts have separate controls.
    report.stats.work.staged_state_capacity_growths = 0;
    report.stats.work.staged_state_capacity_bytes = 0;
    report.stats.work.broad_phase_capacity_growths = 0;
    report.stats.work.candidate_buffer_peak_capacity_bytes = 0;
    // Compare equivalent query work separately from actual preparation/reuse.
    report.stats.work.sweep_bound_preparations += report.stats.work.fixed_sweep_bound_reuses;
    report.stats.work.fixed_sweep_bound_preparations += report.stats.work.fixed_sweep_bound_reuses;
    report.stats.work.fixed_sweep_bound_reuses = 0;
    report.stats.work.fixed_bound_invalidations = 0;
    report.stats.work.fixed_bound_cache_capacity_bytes = 0;
    report
}
