import argparse
import rosdistro

def find_packages_by_prefix(distro_name, prefix):
    index = rosdistro.get_index(rosdistro.get_index_url())
    distro = rosdistro.get_distribution(index, distro_name)
    all_packages = distro.release_packages.keys()
    matched = sorted(pkg for pkg in all_packages if pkg.startswith(prefix))
    return matched

def main():
    parser = argparse.ArgumentParser(description="Find ROS packages by prefix.")
    parser.add_argument('--distro', required=True, help='ROS distribution (e.g., humble, iron)')
    parser.add_argument('--prefix', required=True, help='Package name prefix (e.g., rqt, rviz)')
    args = parser.parse_args()

    packages = find_packages_by_prefix(args.distro, args.prefix)
    if packages:
        print(" ".join(packages))
    else:
        print(f"No packages found with prefix '{args.prefix}' in ROS distro '{args.distro}'.")

if __name__ == '__main__':
    main()
