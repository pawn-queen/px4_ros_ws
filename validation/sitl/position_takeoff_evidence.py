"""独立评估位置模式起飞；真值仅用于离线验收，绝不发送给飞行控制。"""
from __future__ import annotations
import math


def _finite3(v):
    return isinstance(v,(list,tuple)) and len(v)==3 and all(isinstance(x,(int,float)) and math.isfinite(x) for x in v)


def _distance(a,b):
    return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))


def takeoff_evidence(statuses, events, references, navigation, physical, fault=None,
                     injections=(), commands=(), vehicles=(), auto_land_state=18):
    """Check the actual command contract and the strict, measured hover handoff.

    Position-only NaNs are expected only on this explicitly enabled path. A
    successful HOVER_SCAN string alone never satisfies this evaluator.
    """
    states=[s for s in statuses if s.get('px4_takeoff_enabled') is True]
    ascending=[r for r in references if r.get('phase')=='SAFE_TAKEOFF' and r.get('position_only_takeoff')]
    ack=[s for s in states if s.get('px4_takeoff_strict_ack') is True]
    a=ack[0] if ack else {}
    target=next((s.get('px4_takeoff_target_enu_m') for s in states if _finite3(s.get('px4_takeoff_target_enu_m'))),None)
    def reference_enu(r):
        p=r.get('position');return [p[1],p[0],-p[2]] if _finite3(p) else None
    fixed=bool(target and ascending and all(reference_enu(r) is not None and _distance(reference_enu(r),target)<=1e-4
        and math.isfinite(r.get('yaw',math.nan)) and abs(r['yaw']-ascending[0].get('yaw',math.inf))<=1e-5
        and all(math.isnan(x) for x in r['velocity']+r['acceleration']+[r.get('yawspeed',0.)]) for r in ascending))
    strict=[r for r in references if r.get('phase') in ('SAFE_TAKEOFF','HOVER_SCAN')
        and not r.get('position_only_takeoff') and all(math.isfinite(x) for x in r.get('velocity',[])+r.get('acceleration',[]))]
    anchor=a.get('px4_takeoff_anchor_enu_m')
    first=strict[0] if strict else {}
    matching=[r for r in navigation if _finite3(r.get('position')) and r.get('covariance_valid')
        and a.get('mission_status_stamp_ns',0)-300_000_000<=r.get('sample_ns',0)<=a.get('mission_status_stamp_ns',0)]
    anchor_match=bool(_finite3(anchor) and matching and min(_distance(anchor,r['position']) for r in matching)<=.03)
    first_ref=reference_enu(first)
    checks={'position_only_fixed_target_published':fixed and len(ascending)>=10,
        'explicit_strict_ack_observed':bool(a),
        'strict_hold_certificate_valid':bool(a and a.get('motion_mode')=='HOLD'
            and a.get('motion_stop_certificate_valid') is True and a.get('takeoff_safety_stage')=='AIRBORNE_STRICT'),
        'strict_anchor_matches_actual_hover':anchor_match,
        'first_strict_reference_matches_new_anchor':bool(_finite3(anchor) and first_ref and _distance(anchor,first_ref)<=.005),
        'actual_takeoff_and_stable_hover':physical.get('actual_takeoff_and_stable_hover',{}).get('passed') is True,
        'hover_scan_entered_after_ack':any(s.get('phase')=='HOVER_SCAN' and s.get('px4_takeoff_strict_ack') is True for s in states)}
    aborts=[e for e in events if e.get('event')=='px4_takeoff_abort']
    reason=str(aborts[0].get('px4_takeoff_abort_reason','')) if aborts else ''
    reason=reason or next((str(s.get('px4_takeoff_abort_reason')) for s in states if s.get('px4_takeoff_abort_reason')), '')
    fault_result=None
    if fault:
        causal=aborts[0].get('event_stamp_ns',0) if aborts else 0
        nav_land=[c for c in commands if c.get('command')==21 and c.get('sample_ns',0)>=causal]
        auto_land=[s for s in vehicles if s.get('nav_state')==auto_land_state and s.get('sample_ns',0)>=causal]
        fault_checks={'abort_event_observed':bool(aborts),'expected_guard_reason':
            ('horizontal' in reason.lower() or 'drift' in reason.lower()) if fault=='drift' else 'timeout' in reason.lower(),
            'real_nav_land_command_observed':bool(nav_land),'actual_px4_auto_land_observed':bool(auto_land),
            'hover_scan_not_entered':not any(s.get('phase')=='HOVER_SCAN' for s in states),
            'injection_observed':bool(injections) if fault=='drift' else True}
        fault_result={'passed':all(fault_checks.values()),'checks':fault_checks,'requested':fault,
            'abort_reason':reason,'abort_event':aborts[0] if aborts else None,
            'first_nav_land_command':nav_land[0] if nav_land else None,
            'first_actual_auto_land_status':auto_land[0] if auto_land else None,
            'injected_navigation_sample_count':len(injections), 'injected_navigation_samples':list(injections[:10])}
    return {'passed':all(checks.values()),'checks':checks,'fixed_target_enu_m':target,
        'position_only_sample_count':len(ascending),'strict_anchor_enu_m':anchor,
        'first_strict_reference_enu_m':first_ref,'fault_injection':fault_result,
        'scope':'position-only ascent then fresh actual-hover strict HOLD; no mapping/exploration completion claim'}
