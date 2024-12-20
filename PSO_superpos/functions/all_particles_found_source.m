function all_found = all_particles_found_source(particles, current_time)
    % Checks if all particles have found a source
    all_found = all(cellfun(@(p) p.victim_found_flag, particles));
    if all_found
        fprintf('Total simulation time: %.1f minutes\n', current_time/60);
    end
end
