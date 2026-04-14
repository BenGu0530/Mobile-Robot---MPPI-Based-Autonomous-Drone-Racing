# Troubleshooting

## Docker Network Not Found

### Error

```
Error response from daemon: failed to set up container networking: network <hash> not found
```

### Why this happens

Docker is trying to start a container that still references an old network ID which no longer exists.

This usually happens after one of these events:

- containers were removed inconsistently
- a network was pruned or recreated
- old stopped containers still exist with stale network metadata

### Fast recovery (recommended)

Run these commands from the project root:

```bash
docker compose down --remove-orphans

docker rm -f drone_sim gazebo_server 2>/dev/null || true

docker network rm ros_network 2>/dev/null || true

docker network prune -f

export HOST_UID=$(id -u)
export HOST_GID=$(id -g)
docker compose --profile gazebo up --build
```

### Verify it worked

```bash
docker compose ps
docker network ls | grep ros_network
```

Expected:

- `drone_sim` is running
- `gazebo_server` is running (or exits only for a known app/runtime error, not network)
- `ros_network` exists

### If the problem keeps returning

1. Avoid mixing different Compose project contexts for the same files.
2. Keep using `docker compose` (v2) consistently.
3. Keep network name pinned in compose:

```yaml
networks:
  ros_network:
    name: ros_network
    driver: bridge
```

4. If needed, do a full Docker reset for local project artifacts:

```bash
docker compose down --remove-orphans

docker container prune -f
docker network prune -f
```

### Related symptoms and quick checks

- `docker exec -it drone_sim bash` returns exit code 137:
  the container is not running or was killed.

Check:

```bash
docker compose ps
docker compose logs --no-color drone_sim | tail -120
```
