import csv
import json

from calibration.roll_response_analysis import analyze_roll_response


def test_analysis_identifies_negative_physical_roll_mapping(tmp_path):
    run=tmp_path/"run";run.mkdir()
    (run/"metadata.json").write_text(json.dumps({"status":"complete"}))
    phases=["NEUTRAL_BEFORE"]*20+["ROLL_POSITIVE"]*20+["RECOVERY_POSITIVE"]*25+[
        "ROLL_NEGATIVE"]*20+["RECOVERY_NEGATIVE"]*25
    refs={"NEUTRAL_BEFORE":0,"ROLL_POSITIVE":.02,"RECOVERY_POSITIVE":0,
          "ROLL_NEGATIVE":-.02,"RECOVERY_NEGATIVE":0}
    commands=[refs[p] for p in phases];roll=[0,0]+commands[:-2]
    velocity=0.0;rows=[]
    for i,(phase,reference,measured) in enumerate(zip(phases,commands,roll)):
        velocity+=(-5*measured)*.02
        rows.append({"elapsed_s":i*.02,"dt_s":.02,
            "roll_identification_phase":phase,"roll_excitation_reference_rad":reference,
            "roll_rad":measured,"p_rad_s":0 if i==0 else (measured-roll[i-1])/.02,
            "vx_m_s":0,"vy_m_s":velocity,"yaw_rad":0})
    with (run/"flight_telemetry.csv").open("w",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    report=analyze_roll_response(run)
    assert report["command_to_attitude"]["polarity_pass"] is True
    assert report["attitude_to_body_y_acceleration"]["identified_roll_mapping_sign"]==-1
    assert report["result"]=="PASS"
