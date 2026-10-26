import csv
import json

from calibration.pitch_response_analysis import analyze_pitch_response


def test_analysis_separates_command_and_negative_physical_pitch_mapping(tmp_path):
    run=tmp_path/"run";run.mkdir();(run/"metadata.json").write_text(json.dumps({"status":"complete"}))
    phases=["NEUTRAL_BEFORE"]*20+["PITCH_POSITIVE"]*20+["RECOVERY_POSITIVE"]*25+["PITCH_NEGATIVE"]*20+["RECOVERY_NEGATIVE"]*25
    references={"NEUTRAL_BEFORE":0,"PITCH_POSITIVE":.02,"RECOVERY_POSITIVE":0,"PITCH_NEGATIVE":-.02,"RECOVERY_NEGATIVE":0}
    commands=[references[p] for p in phases];pitch=[0,0]+commands[:-2]
    velocity=0.0;rows=[]
    for i,(phase,reference,measured) in enumerate(zip(phases,commands,pitch)):
        acceleration=-5*measured;velocity+=acceleration*.02
        rows.append({"elapsed_s":i*.02,"dt_s":.02,"pitch_identification_phase":phase,
            "pitch_excitation_reference_rad":reference,"pitch_rad":measured,
            "q_rad_s":0 if i==0 else (measured-pitch[i-1])/.02,"vx_m_s":velocity,
            "vy_m_s":0,"yaw_rad":0})
    with (run/"flight_telemetry.csv").open("w",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    report=analyze_pitch_response(run)
    assert report["command_to_attitude"]["polarity_pass"] is True
    assert report["attitude_to_body_x_acceleration"]["identified_pitch_mapping_sign"]==-1
    assert report["result"]=="PASS"
