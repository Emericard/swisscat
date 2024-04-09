<?xml version='1.0'?>
<launch>

<node pkg="swisscat_robot" type="fleet_manager.py" name="fleet_manager" output="screen">
    <env name="PYTHONPATH" value="$(find swisscat_robot)/venv/lib/python3.8/site-packages:$(env PYTHONPATH)"/>
</node>

</launch>
