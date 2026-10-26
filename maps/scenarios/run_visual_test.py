import carla
import time
import numpy as np
from PIL import Image

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)

    world = client.load_world('map_ditch')
    time.sleep(2.0)

    # 1. Weather & Sun angle tuned to emphasize relief and prevent overexposure
    weather = carla.WeatherParameters(
        sun_altitude_angle=30.0,   # Angled sun (30 deg elevation)
        sun_azimuth_angle=65.0,    # Cross-lighting across the ditch for rich slope shadows
        cloudiness=15.0,           # Slight natural cloudiness softens harsh specular glare
        precipitation=0.0,
        precipitation_deposits=0.0,
        wind_intensity=10.0,
        fog_density=0.0,
        fog_distance=0.0,
        wetness=0.0
    )
    world.set_weather(weather)
    print("[OK] Weather & Sun Angle configured.")

    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # Ego: Lincoln MKZ
    ego_bp = bp_lib.filter('vehicle.lincoln.mkz_2017')[0]
    wp_ego = carla_map.get_waypoint(carla.Location(x=50.0, y=5.625, z=0.0))
    ego_tf = wp_ego.transform
    ego_tf.location.z += 1.0
    ego_veh = world.spawn_actor(ego_bp, ego_tf)

    # Target: Audi TT
    target_bp = bp_lib.filter('vehicle.audi.tt')[0]
    wp_target = carla_map.get_waypoint(carla.Location(x=65.0, y=1.875, z=0.0))
    target_tf = wp_target.transform
    target_tf.location.z += 1.0
    target_veh = world.spawn_actor(target_bp, target_tf)

    # Spectator
    spec = world.get_spectator()
    spec_tf = carla.Transform(
        ego_tf.location + carla.Location(x=-14.0, y=1.5, z=4.5),
        carla.Rotation(pitch=-12.0, yaw=0.0, roll=0.0)
    )
    spec.set_transform(spec_tf)

    # Camera with tuned exposure and NO glare/bloom
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', '1280')
    cam_bp.set_attribute('image_size_y', '720')
    cam_bp.set_attribute('fov', '90')
    
    # Anti-glare and crisp exposure settings
    if cam_bp.has_attribute('bloom_intensity'):
        cam_bp.set_attribute('bloom_intensity', '0.05')
    if cam_bp.has_attribute('exposure_compensation'):
        cam_bp.set_attribute('exposure_compensation', '-0.5')
    if cam_bp.has_attribute('lens_flare_intensity'):
        cam_bp.set_attribute('lens_flare_intensity', '0.0')

    cam_tf = carla.Transform(carla.Location(x=-12.0, y=1.5, z=4.0), carla.Rotation(pitch=-12.0, yaw=0.0, roll=0.0))
    cam = world.spawn_actor(cam_bp, cam_tf, attach_to=ego_veh)

    captured = []
    def on_image(img):
        array = np.frombuffer(img.raw_data, dtype=np.uint8)
        array = array.reshape((img.height, img.width, 4))
        captured.append(array[:, :, :3][:, :, ::-1])

    cam.listen(on_image)

    for _ in range(30):
        world.tick()
        time.sleep(0.04)

    if captured:
        img_np = captured[-1]
        im = Image.fromarray(img_np)
        im.save('tuned_visuals_view.png')
        print(f"[OK] Saved tuned view to tuned_visuals_view.png")
        print(f"     Mean RGB: {img_np.mean():.2f}, Road: {img_np[550:650, 450:550].mean(axis=(0,1))}")
        print(f"     Ditch: {img_np[550:650, 950:1100].mean(axis=(0,1))}")
        print(f"     Shoulder: {img_np[550:650, 750:850].mean(axis=(0,1))}")

    cam.stop()
    print("Done!")

if __name__ == '__main__':
    main()
