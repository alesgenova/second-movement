/*
 * MIT License
 *
 * Copyright (c) 2023 Alessandro Genova
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdlib.h>
#include <string.h>
#include "page_ordering_face.h"
#include "movement_config.h"
#include "delay.h"


const char *watch_face_names[] = {
    /* MOVEMENT_FACE_NAME gives a string of the name of the face, which can
     * then be used to get the names of the faces in a user interface.
     */
#define MOVEMENT_FACE_NAME(face_name) #face_name,
    FOREACH_PRIMARY_FACE(MOVEMENT_FACE_NAME)
    FOREACH_SECONDARY_FACE(MOVEMENT_FACE_NAME)
    FOREACH_TERTIARY_FACE(MOVEMENT_FACE_NAME)
};

typedef struct {
    uint8_t watch_face_index;
    uint8_t current_page_index;
    bool touched;
    bool tick_tock;
    bool reordering;
    bool pending_secondary_face;
    bool pending_tertiary_face;
    bool protected;
} page_ordering_face_state_t;

void page_ordering_face_setup(uint8_t watch_face_index, void ** context_ptr) {
    (void) watch_face_index;
    if (*context_ptr == NULL) {
        *context_ptr = malloc(sizeof(page_ordering_face_state_t));
        memset(*context_ptr, 0, sizeof(page_ordering_face_state_t));
        page_ordering_face_state_t *state = (page_ordering_face_state_t *)*context_ptr;
        state->watch_face_index = watch_face_index;
    }
}

void page_ordering_face_activate(void *context) {
    page_ordering_face_state_t *state = (page_ordering_face_state_t *)context;
    state->current_page_index = 0;
    state->touched = false;
    state->tick_tock = true;
    state->reordering = false;
    movement_request_tick_frequency(4);
}

int8_t _section_start(uint8_t page_index) {
    uint8_t secondary_page = movement_get_secondary_page();
    uint8_t tertiary_page = movement_get_tertiary_page();

    if (page_index < secondary_page) {
        return 0; // Primary section
    } else if (page_index < tertiary_page) {
        return secondary_page; // Secondary section
    } else {
        return tertiary_page; // Tertiary section
    }
}

int8_t _section_num(uint8_t page_index) {
    uint8_t secondary_page = movement_get_secondary_page();
    uint8_t tertiary_page = movement_get_tertiary_page();

    if (page_index < secondary_page) {
        return 0; // Primary section
    } else if (page_index < tertiary_page) {
        return 1; // Secondary section
    } else {
        return 2; // Tertiary section
    }
}

void _set_section_place(uint8_t section_num, uint8_t place) {

    switch(section_num) {
        case 1:
            movement_set_secondary_page(place);
            break;
        case 2:
            movement_set_tertiary_page(place);
            break;
        case 0:
            // No further sections
            break;
    }
}

int8_t _section_end(uint8_t page_index) {
    uint8_t secondary_page = movement_get_secondary_page();
    uint8_t tertiary_page = movement_get_tertiary_page();
    uint8_t num_faces = movement_get_num_faces();

    if (page_index < secondary_page) {
        return secondary_page; // Primary section
    } else if (page_index < tertiary_page) {
        return tertiary_page; // Secondary section
    } else {
        return num_faces; // Tertiary section
    }
}

uint8_t _next_section_start(uint8_t page_index) {
    uint8_t secondary_page = movement_get_secondary_page();
    uint8_t tertiary_page = movement_get_tertiary_page();
    uint8_t num_faces = movement_get_num_faces();

    if (page_index < secondary_page) {
        return secondary_page; // Next is Secondary section
    } else if (page_index < tertiary_page) {
        return tertiary_page; // Next is Tertiary section
    } else {
        return 0; // Primary section
    }
}

uint8_t _next_section_end(uint8_t page_index) {
    uint8_t secondary_page = movement_get_secondary_page();
    uint8_t tertiary_page = movement_get_tertiary_page();
    uint8_t num_faces = movement_get_num_faces();

    if (page_index < secondary_page) {
        return tertiary_page; // Next is Tertiary section
    } else if (page_index < tertiary_page) {
        return num_faces; // Next is end of Tertiary section
    } else {
        return secondary_page; // Next is end of Primary section
    }
}


static void _page_ordering_face_update_lcd(page_ordering_face_state_t *state) {
    char buf[11], buf2[4];

    uint8_t section = _section_num(state->current_page_index);

    snprintf(buf, 3, "F%1.1u",1+section);
    snprintf(buf2, 4, "Fo%1.1u", 1 + section);

    watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, buf2, buf);

    // Index of the face associated with the current page
    //snprintf(buf, 4, "%2.1u", state->current_page_index - _section_start(state->current_page_index)+1);

    // Current page
    uint16_t loc_in_current_section = state->current_page_index - _section_start(state->current_page_index)+1 ;
    if (!state->reordering || state->tick_tock) {
        snprintf(buf, 4, "%2u", loc_in_current_section);
    } else {
        snprintf(buf, 3, "  ");
    }
    watch_display_text(WATCH_POSITION_TOP_RIGHT, buf);

    // Whether the page is enabled
    if (movement_is_page_enabled(state->current_page_index))
        watch_set_indicator(WATCH_INDICATOR_SIGNAL);
    else
        watch_clear_indicator(WATCH_INDICATOR_SIGNAL);

    snprintf(buf,7,"%6s", watch_face_names[movement_page_to_face(state->current_page_index)]);
    if( strncmp(
        watch_face_names[movement_page_to_face(state->current_page_index)],
        __FILE_NAME__,  12) == 0 ) {
        state->protected = true; /* Don't allow disabling the page_ordering face*/
    } else {
        state->protected = false;
    }
    watch_display_text(WATCH_POSITION_BOTTOM, buf);
}

static void _page_ordering_face_toggle_page(page_ordering_face_state_t *state) {
    // Prevent this page from ever disabling itself,
    // or else the watch could be in an unrecoverable state
    if (state->current_page_index == movement_face_to_page(state->watch_face_index)) {
        return;
    }

    movement_enable_page(state->current_page_index, !movement_is_page_enabled(state->current_page_index));
    state->touched = true;
}

static void _animate_text(const char *text, int8_t direction) {
    int8_t i, j;
    char buf[8];

    for(i = 0; i < 7; i++) {
        for(j = 0; j < 6; j++) buf[j] = ' ';

        if(direction > 0) {
            /* Copy from position j to end of name to the left of the display */
            for(j = 0; j <= 6 - i; j++) {
                if(text[i + j] == '\0') break;
                buf[j] = text[i + j];
            }
        } else {
            /* copy from position 0 to end-i to the right of the display */
            for(j = 6; j >= i; j--) {
                if(text[j - i] == '\0') break;
                buf[j] = text[j - i];
            }
        }
        buf[6] = '\0';

        watch_display_text(WATCH_POSITION_BOTTOM, buf);
        delay_ms(100);
    }
    delay_ms(50);
}

static void _turn_page(page_ordering_face_state_t *state, int8_t change) {
    // Use uint16_t to avoid integer overflow if num_faces > 127;
    uint16_t start, end;

    start = 0;
    end = movement_get_num_faces();

    uint16_t num_faces = (uint16_t)(end - start);
    uint16_t current_pos = state->current_page_index - start;
    current_pos = ((int32_t)current_pos + change + num_faces) % num_faces;
    uint16_t new_page_index = current_pos + start;

    state->current_page_index = new_page_index;
}

static void _movement_shift_pages(uint8_t from, uint8_t to, int8_t direction) {
    if(direction < 0) {
        for(uint8_t i = from; i < to -1; i++)
            movement_swap_page_order(i, i + 1);
    } else {
        for(uint8_t i = to-1; i > from; i--)
            movement_swap_page_order(i, i - 1);
    }
}


static void _page_ordering(page_ordering_face_state_t *state, int8_t change) {
    // Use uint16_t to avoid integer overflow if num_faces > 127;
    uint16_t start, end;
    uint16_t i;

    start = _section_start(state->current_page_index);
    end = _section_end(state->current_page_index);
    uint16_t num_faces = (uint16_t)(end - start);
    uint16_t current_pos = state->current_page_index - start;
    current_pos = ((int32_t)current_pos + change + num_faces) % num_faces;
    uint16_t new_page_index = current_pos + start;

    _animate_text(watch_face_names[movement_page_to_face(new_page_index)], change);
    if( (state->current_page_index == start) && (change < 0))
        _movement_shift_pages(start, end, change);
    else if((state->current_page_index == end-1) && (change > 0))
        _movement_shift_pages(start, end, change);
    else
        movement_swap_page_order(state->current_page_index, new_page_index);
    state->touched = true;

    state->current_page_index = new_page_index;
}

void _change_face_section(page_ordering_face_state_t *state) {

    uint8_t my_start, my_end, new_start, new_end, my_section;
    uint8_t dest_section_start, dest_section_end;

    my_start = _section_start(state->current_page_index);
    my_end = _section_end(state->current_page_index);
    my_section = _section_num(state->current_page_index);

    dest_section_start = _next_section_start(state->current_page_index);
    dest_section_end = _next_section_end(state->current_page_index);

    printf("Next section start: %u, end: %u\n", dest_section_start, dest_section_end);

    if( state->current_page_index >= dest_section_end) {
        printf("Shorting from %u to %u\n", state->current_page_index, dest_section_end);
        _movement_shift_pages( dest_section_end, state->current_page_index+1, +1);
    } else {
        printf("Shifting from %u to %u\n", state->current_page_index, dest_section_start);
        _movement_shift_pages(state->current_page_index, dest_section_start, -1);
    }

    switch(my_section) {
        case 0:
            // Move to secondary section
            _set_section_place(1, dest_section_start - 1);
            state->current_page_index = dest_section_start - 1;
            break;
        case 1:
            // Move to tertiary section
            _set_section_place(2, dest_section_start - 1);
            state->current_page_index = dest_section_start - 1;
            break;
        case 2:
            // Move to primary section
            _set_section_place(1, dest_section_end + 1);
            _set_section_place(2, my_start + 1);
            state->current_page_index = dest_section_end ;
    }
    // For debugging print location of sections and all pages
    printf("=== After moving Section Debug Info ===\n");
    printf("Secondary page: %u, Tertiary page: %u, Total faces: %u\n",
           movement_get_secondary_page(), movement_get_tertiary_page(), movement_get_num_faces());
    printf("All pages: ");
    for(uint8_t i = 0; i < movement_get_num_faces(); i++) {
        printf("Face %u, ", movement_page_to_face(i));
    }
    printf("\n");
    printf("Section boundaries: %u %u\n", movement_get_secondary_page(), movement_get_tertiary_page());


    state->touched = true;
}

static void _page_ordering_commit_secondary_or_tertiary_face(page_ordering_face_state_t *state) {
    if (state->pending_tertiary_face) {
        movement_set_tertiary_page(state->current_page_index);
    } else {
        movement_set_secondary_page(state->current_page_index);
    }

    state->pending_secondary_face = false;
    state->pending_tertiary_face = false;
    state->touched = true;
}

bool page_ordering_face_loop(movement_event_t event, void *context) {
    page_ordering_face_state_t *state = (page_ordering_face_state_t *)context;

    switch (event.event_type) {
        case EVENT_ACTIVATE:
            break;
        case EVENT_TICK:
            state->tick_tock = !state->tick_tock;
            break;
        case EVENT_ALARM_BUTTON_UP:
            watch_buzzer_play_note(BUZZER_NOTE_C7, 40);
            if( state->reordering)
                _page_ordering(state, +1);
            else
                _turn_page(state, +1);
            break;
        case EVENT_LIGHT_LONG_PRESS:
            if( state->reordering) {
                _change_face_section(state);
            } else {
                if( !state->protected ) {
                    watch_buzzer_play_note(BUZZER_NOTE_C4, 50);
                    _page_ordering_face_toggle_page(state);
                } else {
                    watch_buzzer_play_note(BUZZER_NOTE_C3, 50);
                }

            }
            break;
        case EVENT_ALARM_REALLY_LONG_PRESS:
            if (state->reordering) {
                state->pending_secondary_face = false;
                state->pending_tertiary_face = true;
            }
            break;
        case EVENT_LIGHT_LONG_UP:
            if (state->reordering && (state->pending_secondary_face || state->pending_tertiary_face)) {
                _page_ordering_commit_secondary_or_tertiary_face(state);
                state->reordering = false;
            }
            break;
        case EVENT_LIGHT_BUTTON_UP:
            if (movement_led_stay_off()) {
                movement_force_led_off();
            }
            watch_buzzer_play_note(BUZZER_NOTE_C7, 40);
            if( state->reordering)
                _page_ordering(state, -1);
            else
                _turn_page(state, -1);
            break;
        case EVENT_LIGHT_BUTTON_DOWN:
            movement_illuminate_led();
            break;
        case EVENT_ALARM_LONG_PRESS:
            if (!state->reordering) {
                watch_buzzer_play_note(BUZZER_NOTE_C7, 70);
            } else {
                watch_buzzer_play_note(BUZZER_NOTE_C5, 70);
            }
            state->reordering = !state->reordering;
            break;
        case EVENT_TIMEOUT:
            movement_move_to_page(0);
            break;
        default:
            return movement_default_loop_handler(event);
    }
   _page_ordering_face_update_lcd(state);

    return true;
}

void page_ordering_face_resign(void *context) {
    (void) context;
}
